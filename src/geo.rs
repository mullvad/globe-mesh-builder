use shapefile::dbase::FieldValue;
use shapefile::{Point, PolygonRing, Shape};
use std::f32::consts::{PI, TAU};
use std::hash::Hash;
use std::path::Path;
use vecmat::Vector;

use crate::linalg::Vertex;

const MAX_TRIANGLE_CUT_THROUGH_SPHERE_ERROR: f32 = 0.001;
const MAX_CONTOUR_CUT_THROUGH_SPHERE_ERROR: f32 = MAX_TRIANGLE_CUT_THROUGH_SPHERE_ERROR / 20.0;

/// This is the data format `earcutr` expects. It's conceptually a list of rings.
/// The first ring being the outer ring and any subsequent rings are inner rings.
/// Each ring is a list of coordinates. Each cordinate is two floats [long, lat].
///
///                             ___ - two floats, [long, lat]. But represented with a Vec sadly.
///                         ___ - Represents a ring
///                     ___ - A list of rings. The first ring is outer, the remaining inner.
///                           Together these form a polygon with holes
type PolygonWithHoles = Vec<Vec<Vec<f64>>>;

#[derive(Copy, Clone, Debug)]
pub struct Coordinate {
    pub lat: f32,
    pub long: f32,
}

fn f32_round(f: f32) -> i64 {
    (f * 10_000.0).round() as i64
}

impl PartialEq for Coordinate {
    fn eq(&self, other: &Self) -> bool {
        f32_round(self.lat) == f32_round(other.lat) && f32_round(self.long) == f32_round(other.long)
    }
}

impl Eq for Coordinate {}

impl Hash for Coordinate {
    fn hash<H: std::hash::Hasher>(&self, state: &mut H) {
        f32_round(self.lat).hash(state);
        f32_round(self.long).hash(state);
    }
}

/// An 2D triangle with geo coordinates as radians
#[derive(Copy, Clone, Debug, Eq, PartialEq, Hash)]
pub struct Triangle([Coordinate; 3]);

impl Triangle {
    pub fn to_coordinates(self) -> [Coordinate; 3] {
        self.0
    }
}

impl From<[Coordinate; 3]> for Triangle {
    fn from(vertices: [Coordinate; 3]) -> Self {
        Triangle(vertices)
    }
}

/// Takes a line consisting of multiple coordinates and splits each segment (line)
/// that is too long. Prevents contours from taking a shortcut through the globe too much
pub fn subdivide_contour(contour: &[Coordinate]) -> Vec<Coordinate> {
    /// Returns a list of coordinates making up the line between c0 and c1 (excluding c1 itself)
    /// but split into small enough chunks.
    fn split_line(c0: Coordinate, c1: Coordinate) -> Vec<Coordinate> {
        let line_error = error(c0, c1);
        if line_error > MAX_CONTOUR_CUT_THROUGH_SPHERE_ERROR {
            let midpoint = midpoint(c0, c1);
            let mut output = split_line(c0, midpoint);
            output.extend(split_line(midpoint, c1));
            output
        } else {
            // Base case. Line is not too long. So return the first coordinate
            vec![c0]
        }
    }

    let mut output_contour = Vec::with_capacity(contour.len());
    for &[c0, c1] in contour.array_windows::<2>() {
        output_contour.extend(split_line(c0, c1));
    }
    // Since `split_line` does not include the last coordinate, we add it manually
    if let Some(last_coordinate) = contour.last() {
        output_contour.push(*last_coordinate);
    }
    output_contour
}

/// Takes a geo triangle as input, looks at the lat and long angles between the coordinates
/// and cuts all edges in half that has one angle larger than MAX_COORDINATE_ANGLE_RAD
/// and recurse until no angle is too large.
pub fn subdivide_triangle(triangle: Triangle) -> Vec<Triangle> {
    log::debug!("Subdividing {:?}", triangle);
    let [c0, c1, c2] = triangle.to_coordinates();
    if c0 == c1 || c1 == c2 || c2 == c0 {
        log::debug!("Triangle with zero area!");
        return vec![triangle];
    }

    let c0c1_error = error(c0, c1);
    let c1c2_error = error(c1, c2);
    let c2c0_error = error(c2, c0);

    let mut too_long_side = false;
    if c0c1_error > MAX_TRIANGLE_CUT_THROUGH_SPHERE_ERROR {
        too_long_side = true;
        log::debug!("c0-c1 is too long: {c0c1_error}");
    }
    if c1c2_error > MAX_TRIANGLE_CUT_THROUGH_SPHERE_ERROR {
        too_long_side = true;
        log::debug!("c1-c2 is too long: {c1c2_error}");
    }
    if c2c0_error > MAX_TRIANGLE_CUT_THROUGH_SPHERE_ERROR {
        too_long_side = true;
        log::debug!("c2-c0 is too long: {c2c0_error}");
    }
    // Base case. Triangle is not large enough to need subdivision. Stop recursion.
    if !too_long_side {
        return vec![triangle];
    }

    // At least one edge is too long. Rotate it so that it has the longest
    // side furthes away from c0, to fit with `split_triangle`
    let rotated_triangle = if c0c1_error >= c1c2_error && c0c1_error >= c2c0_error {
        // The side between c0 and c1 is the longest
        Triangle::from([c2, c0, c1])
    } else if c1c2_error >= c2c0_error && c1c2_error >= c0c1_error {
        // The side between c1 and c2 is the longest
        Triangle::from([c0, c1, c2])
    } else if c2c0_error >= c0c1_error && c2c0_error >= c1c2_error {
        // The side between c2 and c0 is the longest
        Triangle::from([c1, c2, c0])
    } else {
        unreachable!("One edge has to be the longest")
    };

    // `output` will have at least two entries.
    let mut output = Vec::with_capacity(2);

    let [new_triangle1, new_triangle2] = split_triangle(rotated_triangle);
    output.extend(subdivide_triangle(new_triangle1));
    output.extend(subdivide_triangle(new_triangle2));
    output
}

/// Returns how far from earth's surface the 3D line between these coordinates deviates.
/// Since a line when translated to 3D should ideally follow the surface, if it
/// deviates too far, it should be split into multiple segments to reduce the error.
fn error(c0: Coordinate, c1: Coordinate) -> f32 {
    let v0 = latlong2xyz(c0).to_vector();
    let v1 = latlong2xyz(c1).to_vector();

    let midpoint = midpoint_3d(v0, v1);

    // The surface is always at distance 1.0 (sphere), so the error is the difference from that
    (1.0 - midpoint.length()).abs()
}

/// Maps a geographical coordinate represented in radians onto a sphere
/// with radius 1.0, and returns the 3D vector
pub fn latlong2xyz(c: Coordinate) -> Vertex {
    // Polar angle. 0 <= φ <= PI = colatitude in geography
    let phi = (PI / 2.0) - c.lat;
    // Azimuthal angle = longitude. -PI <= θ <= PI
    let theta = c.long;

    let x = phi.sin() * theta.sin();
    let y = phi.cos();
    let z = phi.sin() * theta.cos();
    Vertex::from([x, y, z])
}

/// Splits a triangle into two. The edge that is cut into two is the one between c1 and c2.
fn split_triangle(triangle: Triangle) -> [Triangle; 2] {
    let [c0, c1, c2] = triangle.to_coordinates();
    assert_ne!(c1, c2);
    let c1c2_midpoint = midpoint(c1, c2);
    [
        Triangle::from([c0, c1, c1c2_midpoint]),
        Triangle::from([c0, c1c2_midpoint, c2]),
    ]
}

/// Returns the 3D point halfway between v0 and v1.
fn midpoint_3d(v0: Vector<f32, 3>, v1: Vector<f32, 3>) -> Vector<f32, 3> {
    (v0 + v1) / 2.0
}

/// Returns the coordinate halfway between c0 and c1, taking the shortest
/// route between the two.
fn midpoint(mut c0: Coordinate, mut c1: Coordinate) -> Coordinate {
    clean_coordinate(&mut c0);
    clean_coordinate(&mut c1);
    let mut d_long = c1.long - c0.long;
    if d_long > PI {
        d_long -= TAU;
    } else if d_long < -PI {
        d_long += TAU;
    }
    let mut output = Coordinate {
        lat: (c0.lat + c1.lat) / 2.0,
        long: c0.long + (d_long / 2.0),
    };
    clean_coordinate(&mut output);
    output
}

fn clean_coordinate(c: &mut Coordinate) {
    if c.long <= -PI {
        c.long += TAU;
    } else if c.long > PI {
        c.long -= TAU;
    }
    assert!(c.long > -PI && c.long <= PI);
}

#[cfg(test)]
mod tests {
    use super::{latlong2xyz, midpoint, Coordinate};

    fn equal_float(f0: f32, f1: f32) -> bool {
        (f0 - f1).abs() <= crate::MIN_2D_POLAR_COORDINATE_ERROR
    }

    fn assert_equal_midpoint(c0: Coordinate, c1: Coordinate, expected: Coordinate) {
        let midpoint01 = midpoint(c0, c1);
        let midpoint10 = midpoint(c1, c0);
        assert!(equal_float(midpoint01.lat, midpoint10.lat));
        assert!(equal_float(dbg!(midpoint01.long), dbg!(midpoint10.long)));

        assert!(equal_float(midpoint01.lat, expected.lat));
        assert!(equal_float(midpoint01.long, expected.long));
    }

    #[test]
    fn midpoint_179() {
        let c0 = Coordinate {
            lat: 0.0,
            long: -171.0f32.to_radians(),
        };
        let c1 = Coordinate {
            lat: 0.0,
            long: 170.0f32.to_radians(),
        };
        assert_equal_midpoint(
            c0,
            c1,
            Coordinate {
                lat: 0.0,
                long: 179.5f32.to_radians(),
            },
        );
    }

    #[test]
    fn midpoint_0() {
        let c0 = Coordinate {
            lat: 0.0,
            long: 0.0f32.to_radians(),
        };
        let c1 = Coordinate {
            lat: 0.0,
            long: 180.0f32.to_radians(),
        };
        assert_equal_midpoint(
            c0,
            c1,
            Coordinate {
                lat: 0.0,
                long: 90.0f32.to_radians(),
            },
        );
    }

    #[test]
    fn midpoint_180_meridian() {
        let c0 = Coordinate {
            lat: 0.0,
            long: -180f32.to_radians(),
        };
        let c1 = Coordinate {
            lat: 0.0,
            long: 180.0f32.to_radians(),
        };
        assert_equal_midpoint(
            c0,
            c1,
            Coordinate {
                lat: 0.0,
                long: 180.0f32.to_radians(),
            },
        );
    }

    #[test]
    fn latlong2xyz_sanity() {
        let north_pole1 = geo::Coordinate {
            lat: PI / 2.0,
            long: 0.0,
        };
        let north_pole2 = geo::Coordinate {
            lat: PI / 2.0,
            long: PI / 1.2,
        };
        assert_eq!(latlong2xyz(north_pole1), Vertex::from([0.0, 1.0, 0.0]));
        assert_eq!(latlong2xyz(north_pole2), Vertex::from([0.0, 1.0, 0.0]));

        let south_pole1 = geo::Coordinate {
            lat: -PI / 2.0,
            long: 0.0,
        };
        let south_pole2 = geo::Coordinate {
            lat: -PI / 2.0,
            long: 0.1234,
        };
        assert_eq!(latlong2xyz(south_pole1), Vertex::from([0.0, -1.0, 0.0]));
        assert_eq!(latlong2xyz(south_pole2), Vertex::from([0.0, -1.0, 0.0]));

        let zero = geo::Coordinate {
            lat: 0.0,
            long: 0.0,
        };
        assert_eq!(latlong2xyz(zero), Vertex::from([0.0, 0.0, 1.0]));

        let backside_equator = geo::Coordinate { lat: 0.0, long: PI };
        assert_eq!(
            latlong2xyz(backside_equator),
            Vertex::from([0.0, 0.0, -1.0])
        );

        let to_the_right = geo::Coordinate {
            lat: 0.0,
            long: PI / 2.0,
        };
        assert_eq!(latlong2xyz(to_the_right), Vertex::from([1.0, 0.0, 0.0]));

        let to_the_left = geo::Coordinate {
            lat: 0.0,
            long: -PI / 2.0,
        };
        assert_eq!(latlong2xyz(to_the_left), Vertex::from([-1.0, 0.0, 0.0]));
    }
}

pub fn read_world(path: impl AsRef<Path>) -> (Vec<Triangle>, Vec<Vec<Coordinate>>) {
    let mut reader = shapefile::Reader::from_path(path).unwrap();

    let mut vertices = Vec::new();
    let mut contours = Vec::new();

    for result in reader.iter_shapes_and_records() {
        let (shape, record) = result.unwrap();
        // log::debug!("Shape: {}, records:", shape);
        // for (name, value) in record {
        //     log::debug!("\t{}: {:?}, ", name, value);
        // }
        let country_name = match record.get("NAME") {
            Some(&FieldValue::Character(Some(ref name))) => name,
            _ => "N/A",
        };
        // FIXME: There is a triangle subdivide bug currently not allowing subdivision of
        // triangles spanning over poles. As a quick hack we remove Antarctica until that's fixed
        match record.get("CONTINENT") {
            Some(&FieldValue::Character(Some(ref continent))) if continent == "Antarctica" => {
                log::debug!("Skipping antarctica");
                continue;
            }
            _ => (),
        }

        fn ring_points_to_vec(points: &[Point]) -> Vec<Vec<f64>> {
            points.iter().map(|p| vec![p.x, p.y]).collect::<Vec<_>>()
        }

        fn ring_points_to_coordinates(points: &[Point]) -> Vec<Coordinate> {
            points
                .iter()
                .map(|p| Coordinate {
                    lat: p.y.to_radians() as f32,
                    long: p.x.to_radians() as f32,
                })
                .collect::<Vec<_>>()
        }

        match shape {
            Shape::Polygon(polygon) => {
                let mut rings = PolygonWithHoles::new();
                for ring in polygon.rings() {
                    match ring {
                        PolygonRing::Outer(points) => {
                            if rings.is_empty() {
                                // First ring in a shape is always the outer ring. But don't
                                // compute triangles yet, since it might contain inner rings.
                                rings.push(ring_points_to_vec(points));
                            } else {
                                // An outer ring means we start on a new shape. So compute
                                if let Ok(triangles) = process_polygon_with_holes(&rings) {
                                    vertices.extend(triangles);
                                } else {
                                    log::error!("Failed to process a polygon for {country_name}");
                                }
                                rings.clear();
                                rings.push(ring_points_to_vec(points));
                            }
                            contours.push(ring_points_to_coordinates(points));
                        }
                        PolygonRing::Inner(points) => {
                            rings.push(ring_points_to_vec(points));
                            contours.push(ring_points_to_coordinates(points));
                        }
                    }
                }
                // If we have accumulated rings without making triangles out of them,
                // do it now. This entire loop should really be rewritten into something better.
                if !rings.is_empty() {
                    if let Ok(triangles) = process_polygon_with_holes(&rings) {
                        vertices.extend(triangles);
                    } else {
                        log::error!("Failed to process a polygon for {country_name}");
                    }
                }
            }
            Shape::Polyline(line) => {
                for part in line.parts() {
                    contours.push(ring_points_to_coordinates(part));
                }
            }
            _ => unimplemented!(),
        }
    }
    (vertices, contours)
}

/// Takes a 2D polygon, performs earcutr and returns the 2D triangles with radian coordinates.
fn process_polygon_with_holes(polygon: &PolygonWithHoles) -> Result<Vec<Triangle>, earcutr::Error> {
    let (flat_vertices, hole_indices, dims) = earcutr::flatten(polygon);
    assert_eq!(dims, 2);
    // Convert coordinates to radians and lower resolution to f32 and use for the rest of the program
    let flat_vertices = flat_vertices
        .into_iter()
        .map(|f: f64| f.to_radians() as f32)
        .collect::<Vec<_>>();

    let triangle_vertice_start_indices = earcutr::earcut(&flat_vertices, &hole_indices, dims)?;
    let mut output = Vec::with_capacity(triangle_vertice_start_indices.len() / 3);
    for &[i, j, k] in triangle_vertice_start_indices.as_chunks::<3>().0 {
        let c0 = Coordinate {
            lat: flat_vertices[i * dims + 1],
            long: flat_vertices[i * dims],
        };
        let c1 = Coordinate {
            lat: flat_vertices[j * dims + 1],
            long: flat_vertices[j * dims],
        };
        let c2 = Coordinate {
            lat: flat_vertices[k * dims + 1],
            long: flat_vertices[k * dims],
        };
        let triangle = Triangle::from([c0, c1, c2]);
        output.push(triangle);
    }
    Ok(output)
}
