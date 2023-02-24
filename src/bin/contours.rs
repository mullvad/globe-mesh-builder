#![feature(array_windows)]

use geojson_processor::geo::Coordinate;
use geojson_processor::linalg::Vertex;
use std::collections::hash_map::Entry;
use std::collections::HashMap;
use std::f32::consts::PI;
use std::fs;
use std::path::Path;
use vecmat::Vector;

use geojson::{GeoJson, Geometry, PolygonType, Value};

fn parse_geojson(path: impl AsRef<Path>) -> GeoJson {
    let geojson_str = fs::read_to_string(path).unwrap();
    geojson_str.parse::<GeoJson>().unwrap()
}

/// The structur this program outputs (in JSON format).
#[derive(Debug, serde::Serialize)]
struct Output {
    /// A vector with vertice float values for OpenGL to render. each group of three
    /// floats is one vertice. So `(positions[x], positions[x+1], positions[x+2])` is
    /// one vertice.
    positions: Vec<f32>,
    indices: Vec<u32>,
}

fn main() {
    let geo = parse_geojson(std::env::args().nth(1).unwrap());
    let mut contours = Vec::new();
    match geo {
        GeoJson::FeatureCollection(ref ctn) => {
            for feature in &ctn.features {
                if let Some(geometry) = &feature.geometry {
                    contours.extend(geometry_to_country_contours(&geometry));
                }
            }
        }
        GeoJson::Feature(ref feature) => {
            if let Some(geometry) = &feature.geometry {
                contours.extend(geometry_to_country_contours(&geometry));
            }
        }
        GeoJson::Geometry(geometry) => contours.extend(geometry_to_country_contours(&geometry)),
    }

    let mut seen_vertices: HashMap<Vertex, u32> = HashMap::new();
    let mut output = Output {
        positions: Vec::new(),
        indices: Vec::new(),
    };
    let mut next_index: u32 = 0;
    for contour in contours {
        for line_vertices in contour.array_windows::<2>() {
            for &vertex in line_vertices {
                match seen_vertices.entry(vertex) {
                    Entry::Occupied(entry) => {
                        // This vertex is already in `output.positions`,
                        // just push the index
                        output.indices.push(*entry.get());
                    }
                    Entry::Vacant(entry) => {
                        entry.insert(next_index);
                        output.indices.push(next_index);
                        output.positions.extend(vertex.to_vector().into_array());
                        next_index = next_index.checked_add(1).unwrap();
                    }
                }
            }
        }
    }

    let stdout = std::io::stdout().lock();
    serde_json::to_writer(stdout, &output).unwrap();
}

fn geometry_to_country_contours(geom: &Geometry) -> Vec<Vec<Vertex>> {
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

fn polygon_to_contours(polygon: &PolygonType) -> Vec<Vec<Vertex>> {
    let mut contours = Vec::with_capacity(polygon.len());
    for ring in polygon {
        let mut contour = Vec::with_capacity(ring.len());
        for coordinate in ring {
            let [long, lat] = <[f64; 2]>::try_from(coordinate.as_slice()).unwrap();
            contour.push(latlong2xyz(Coordinate {
                lat: lat.to_radians() as f32,
                long: long.to_radians() as f32,
            }));
        }
        contours.push(contour)
    }
    contours
}

/// Maps a geographical coordinate represented in radians onto a sphere
/// with radius 1.0, and returns the 3D vector
fn latlong2xyz(c: Coordinate) -> Vertex {
    // Polar angle. 0 <= φ <= PI = colatitude in geography
    let phi = (PI / 2.0) - c.lat;
    // Azimuthal angle = longitude. 0 <= θ <= 2*PI
    let theta = c.long + PI;
    let x = -(phi.sin() * theta.cos());
    let z = phi.sin() * theta.sin();
    let y = phi.cos();
    Vertex::from(Vector::from_array([x, y, z]))
}
