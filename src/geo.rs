use geojson::{GeoJson, Geometry, Value};
use std::convert::TryFrom;
use std::fs;
use std::path::Path;
use vecmat::Vector;
use crate::linalg::Triangle;

/// Reads GeoJSON from a file, triangulates the mesh with the earcut algorithm
/// and returns a vector of 2D triangles with coordinates in radians
pub fn triangulate_geojson(path: impl AsRef<Path>) -> Vec<Triangle<f32, 2>> {
    let geojson = parse_geojson(path);
    geojson_to_triangles(&geojson)
}

fn parse_geojson(path: impl AsRef<Path>) -> GeoJson {
    let geojson_str = fs::read_to_string(path).unwrap();
    geojson_str.parse::<GeoJson>().unwrap()
}

/// Process top-level GeoJSON items and returns a vector of 2D triangles
/// with the coordinates in radians
fn geojson_to_triangles(gj: &GeoJson) -> Vec<Triangle<f32, 2>> {
    let mut vertices = Vec::new();
    match *gj {
        GeoJson::FeatureCollection(ref ctn) => {
            for feature in &ctn.features {
                if let Some(ref geom) = feature.geometry {
                    vertices.extend(geometry_to_triangles(geom));
                }
            }
        }
        GeoJson::Feature(ref feature) => {
            if let Some(ref geom) = feature.geometry {
                vertices.extend(geometry_to_triangles(geom));
            }
        }
        GeoJson::Geometry(ref geometry) => vertices.extend(geometry_to_triangles(geometry)),
    }
    vertices
}

/// Process GeoJSON geometries and returns a vector of 2D triangles with
/// the coordinates in radians.
fn geometry_to_triangles(geom: &Geometry) -> Vec<Triangle<f32, 2>> {
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
fn process_polygon_with_holes(polygon: &geojson::PolygonType) -> Vec<Triangle<f32, 2>> {
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
        let v0 = Vector::<f32, 2>::try_from(&flat_vertices[i * dims..i * dims + 2]).unwrap();
        let v1 = Vector::<f32, 2>::try_from(&flat_vertices[j * dims..j * dims + 2]).unwrap();
        let v2 = Vector::<f32, 2>::try_from(&flat_vertices[k * dims..k * dims + 2]).unwrap();
        let triangle = Triangle::from([v0, v1, v2]);
        output.push(triangle);
    }
    output
}
