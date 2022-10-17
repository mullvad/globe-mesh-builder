use geojson::{GeoJson, Geometry, PolygonType, Value};
use std::fs;
use std::path::Path;

const MAX_EDGE_LENGTH_KM: f32 = 250.0;

#[derive(Copy, Clone, Debug)]
pub struct Coordinate {
    pub lat: f32,
    pub long: f32,
}

/// An 2D triangle with geo coordinates as radians
#[derive(Copy, Clone, Debug)]
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

/// Reads GeoJSON from a file, triangulates the mesh with the earcut algorithm
/// and returns a vector of 2D triangles with coordinates in radians
pub fn triangulate_geojson(path: impl AsRef<Path>) -> (Vec<Triangle>, Vec<Vec<Coordinate>>) {
    let geojson = parse_geojson(path);
    geojson_to_triangles(&geojson)
}

/// Takes a geo triangle as input, finds the longest side of it and if that is longer
/// than MAX_EDGE_LENGTH_KM in haversine distance, cut it into two, and recurse.
pub fn subdivide_triangle(triangle: Triangle) -> Vec<Triangle> {
    let [c0, c1, c2] = triangle.to_coordinates();
    let d01 = haversine_distance(c0, c1);
    let d12 = haversine_distance(c1, c2);
    let d20 = haversine_distance(c2, c0);

    if d01 <= MAX_EDGE_LENGTH_KM && d12 <= MAX_EDGE_LENGTH_KM && d20 <= MAX_EDGE_LENGTH_KM {
        // No side is too long. Base case, no triangle splitting will happen
        return vec![triangle];
    }

    // At least one edge is too long. Rotate it so that it has the longest
    // side furthes away from c0, to fit with `split_triangle`
    let rotated_triangle = if d01 >= d12 && d01 >= d20 {
        // The side between c0 and c1 is the longest
        Triangle::from([c2, c0, c1])
    } else if d12 >= d20 && d12 > d01 {
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
    output.extend(subdivide_triangle(new_triangle1));
    output.extend(subdivide_triangle(new_triangle2));
    output
}

/// Splits a triangle into two. The edge that is cut into two is the one between c1 and c2.
fn split_triangle(triangle: Triangle) -> [Triangle; 2] {
    let [c0, c1, c2] = triangle.to_coordinates();
    let c1c2_midpoint = Coordinate {
        lat: (c1.lat + c2.lat) / 2.0,
        long: (c1.long + c2.long) / 2.0,
    };
    [
        Triangle::from([c0, c1, c1c2_midpoint]),
        Triangle::from([c0, c1c2_midpoint, c2]),
    ]
}

fn parse_geojson(path: impl AsRef<Path>) -> GeoJson {
    let geojson_str = fs::read_to_string(path).unwrap();
    geojson_str.parse::<GeoJson>().unwrap()
}

/// Process top-level GeoJSON items and returns a vector of 2D triangles
/// with the coordinates in radians
fn geojson_to_triangles(gj: &GeoJson) -> (Vec<Triangle>, Vec<Vec<Coordinate>>) {
    let mut vertices = Vec::new();
    let mut contours = Vec::new();
    match *gj {
        GeoJson::FeatureCollection(ref ctn) => {
            for feature in &ctn.features {
                if let Some(geometry) = &feature.geometry {
                    vertices.extend(geometry_to_triangles(&geometry));
                    contours.extend(geometry_to_country_contours(&geometry));
                }
            }
        }
        GeoJson::Feature(ref feature) => {
            if let Some(geometry) = &feature.geometry {
                vertices.extend(geometry_to_triangles(geometry));
                contours.extend(geometry_to_country_contours(&geometry));
            }
        }
        GeoJson::Geometry(ref geometry) => {
            vertices.extend(geometry_to_triangles(geometry));
            contours.extend(geometry_to_country_contours(&geometry));
        },
    }
    (vertices, contours)
}

/// Process GeoJSON geometries and returns a vector of 2D triangles with
/// the coordinates in radians.
fn geometry_to_triangles(geom: &Geometry) -> Vec<Triangle> {
    let mut vertices = Vec::new();
    match &geom.value {
        Value::Polygon(p) => {
            log::debug!("Matched a Polygon");
            vertices.extend(process_polygon_with_holes(p));
        }
        Value::MultiPolygon(polygons) => {
            log::debug!("Matched a MultiPolygon");
            for p in polygons {
                vertices.extend(process_polygon_with_holes(p));
            }
        }
        Value::GeometryCollection(ref gc) => {
            log::debug!("Matched a GeometryCollection");
            // GeometryCollections contain other Geometry types, and can nest
            // we deal with this by recursively processing each geometry
            for geometry in gc {
                vertices.extend(geometry_to_triangles(geometry))
            }
        }
        // Point, LineString, and their Multi– counterparts
        _ => todo!("Matched some other geometry"),
    }
    vertices
}

/// Takes a 2D polygon, performs earcutr and returns the 2D triangles with radian coordinates.
fn process_polygon_with_holes(polygon: &geojson::PolygonType) -> Vec<Triangle> {
    let (flat_vertices, hole_indices, dims) = earcutr::flatten(polygon);
    assert_eq!(dims, 2);
    // Convert coordinates to radians and lower resolution to f32 and use for the rest of the program
    let flat_vertices = flat_vertices
        .into_iter()
        .map(|f: f64| f.to_radians() as f32)
        .collect::<Vec<_>>();

    let triangle_vertice_start_indices = earcutr::earcut(&flat_vertices, &hole_indices, dims);
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
    output
}

fn geometry_to_country_contours(geom: &Geometry) -> Vec<Vec<Coordinate>> {
    let mut contours = Vec::new();
    match &geom.value {
        Value::Polygon(polygon) => {
            log::debug!("Matched a Polygon");
            contours.extend(polygon_to_contours(polygon));
        }
        Value::MultiPolygon(polygons) => {
            log::debug!("Matched a MultiPolygon");
            for polygon in polygons {
                contours.extend(polygon_to_contours(polygon));
            }
        }
        Value::GeometryCollection(ref _gc) => {
            unimplemented!("Matched a GeometryCollection");
        }
        // Point, LineString, and their Multi– counterparts
        _ => unimplemented!("Matched some other geometry"),
    }
    contours
}

fn polygon_to_contours(polygon: &PolygonType) -> Vec<Vec<Coordinate>> {
    let mut contours = Vec::with_capacity(polygon.len());
    for ring in polygon {
        let mut contour = Vec::with_capacity(ring.len());
        for coordinate in ring {
            let [long, lat] = <[f64; 2]>::try_from(coordinate.as_slice()).unwrap();
            contour.push(Coordinate {
                lat: lat.to_radians() as f32,
                long: long.to_radians() as f32,
            });
        }
        contours.push(contour)
    }
    contours
}

/// Implemented as per https://en.wikipedia.org/wiki/Haversine_formula
/// and https://rosettacode.org/wiki/Haversine_formula#Rust
/// Takes input as radians, outputs kilometers.
fn haversine_distance(c0: Coordinate, c1: Coordinate) -> f32 {
    const RAIDUS_OF_EARTH: f32 = 6372.8;

    let d_lat = c0.lat - c1.lat;
    let d_lon = c0.long - c1.long;
    // Computing the haversine between two points
    let haversine =
        (d_lat / 2.0).sin().powi(2) + (d_lon / 2.0).sin().powi(2) * c0.lat.cos() * c1.lat.cos();

    // using the haversine to compute the distance between two points
    haversine.sqrt().asin() * 2.0 * RAIDUS_OF_EARTH
}
