use shapefile::dbase::FieldValue;
use shapefile::{Point, PolygonRing, Shape};
use std::collections::HashSet;
use std::f32::consts::{PI, TAU};
use std::hash::Hash;
use std::path::Path;

const MAX_COORDINATE_ANGLE_RAD: f32 = (5.0 / 180.0) * PI;

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

/// Takes a geo triangle as input, looks at the lat and long angles between the coordinates
/// and cuts all edges in half that has one angle larger than MAX_COORDINATE_ANGLE_RAD
/// and recurse until no angle is too large.
pub fn subdivide_triangle(
    triangle: Triangle,
    seen_triangles: &mut HashSet<Triangle>,
) -> Vec<Triangle> {
    log::debug!("Subdividing {:?}", triangle);
    let [c0, c1, c2] = triangle.to_coordinates();
    if c0 == c1 || c1 == c2 || c2 == c0 {
        log::debug!("Triangle with zero area!");
        return vec![triangle];
    }

    if !seen_triangles.insert(triangle) {
        panic!("We have already seen {triangle:?}");
    }

    fn largest_angle(c0: Coordinate, c1: Coordinate) -> f32 {
        let d_lat = (c1.lat - c0.lat).abs();

        let mut d_long = c1.long - c0.long;
        if d_long > PI {
            d_long -= TAU;
        } else if d_long < -PI {
            d_long += TAU;
        }

        d_lat.max(d_long.abs())
    }
    let d01 = largest_angle(c0, c1);
    let d12 = largest_angle(c1, c2);
    let d20 = largest_angle(c2, c0);

    let mut too_long_side = false;
    if d01 > MAX_COORDINATE_ANGLE_RAD {
        too_long_side = true;
        log::debug!("c0-c1 is too long: {d01}");
    }
    if d12 > MAX_COORDINATE_ANGLE_RAD {
        too_long_side = true;
        log::debug!("c1-c2 is too long: {d12}");
    }
    if d20 > MAX_COORDINATE_ANGLE_RAD {
        too_long_side = true;
        log::debug!("c2-c0 is too long: {d20}");
    }
    // Base case. Triangle is not large enough to need subdivision. Stop recursion.
    if !too_long_side {
        return vec![triangle];
    }

    // At least one edge is too long. Rotate it so that it has the longest
    // side furthes away from c0, to fit with `split_triangle`
    let rotated_triangle = if d01 >= d12 && d01 >= d20 {
        // The side between c0 and c1 is the longest
        Triangle::from([c2, c0, c1])
    } else if d12 >= d20 && d12 >= d01 {
        // The side between c1 and c2 is the longest
        Triangle::from([c0, c1, c2])
    } else if d20 >= d01 && d20 >= d12 {
        // The side between c2 and c0 is the longest
        Triangle::from([c1, c2, c0])
    } else {
        unreachable!("One edge has to be the longest")
    };

    // `output` will have at least two entries.
    let mut output = Vec::with_capacity(2);

    let [new_triangle1, new_triangle2] = split_triangle(rotated_triangle);
    output.extend(subdivide_triangle(new_triangle1, seen_triangles));
    output.extend(subdivide_triangle(new_triangle2, seen_triangles));
    output
}

/// Splits a triangle into two. The edge that is cut into two is the one between c1 and c2.
fn split_triangle(triangle: Triangle) -> [Triangle; 2] {
    let [c0, c1, c2] = triangle.to_coordinates();
    let c1c2_midpoint = midpoint(c1, c2);
    [
        Triangle::from([c0, c1, c1c2_midpoint]),
        Triangle::from([c0, c1c2_midpoint, c2]),
    ]
}

fn midpoint(c0: Coordinate, c1: Coordinate) -> Coordinate {
    assert_ne!(c0, c1);
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
    assert_ne!(output, c0);
    assert_ne!(output, c1);
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
    use super::{midpoint, Coordinate};

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
