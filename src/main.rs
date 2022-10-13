#![feature(array_chunks)]
#![feature(slice_as_chunks)]

use clap::Parser;
use geojson::{GeoJson, Geometry, Value};
use num_traits::float::Float;
use std::convert::TryFrom;
use std::fs;
use std::path::{Path, PathBuf};
use vecmat::Vector;

#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
    /// Path to the GeoJSON input file to process
    #[arg(short, long)]
    geojson: PathBuf,

    /// If the output should be pretty json or not
    #[arg(short, long)]
    pretty: bool,
}

fn main() {
    let args = Args::parse();
    let mesh: Vec<Vertex> = world_vertices(args.geojson);
    let json_data = if args.pretty {
        serde_json::to_string_pretty(&mesh).unwrap()
    } else {
        serde_json::to_string(&mesh).unwrap()
    };
    println!("{}", json_data);
}

/// A struct representing a single vertex in 3D space with a normal.
///
/// The `repr(C)` attribute is required to ensure that the memory layout is
/// what we expect. Without it, no specific layout is guaranteed.
#[derive(Copy, Clone, Debug, serde::Serialize)]
#[repr(C)]
pub struct Vertex {
    position: [f32; 3],
    normal: [f32; 3],
}

#[derive(Copy, Clone, Debug)]
pub struct Triangle<T: Float, const N: usize>([Vector<T, N>; 3]);

impl<T: Float, const N: usize> Triangle<T, N> {
    pub fn vertices(&self) -> &[Vector<T, N>; 3] {
        &self.0
    }
}

impl<T: Float, const N: usize> From<[Vector<T, N>; 3]> for Triangle<T, N> {
    fn from(vertices: [Vector<T, N>; 3]) -> Self {
        Triangle(vertices)
    }
}

impl<T: Float, const N: usize> From<Triangle<T, N>> for [Vector<T, N>; 3] {
    fn from(triangle: Triangle<T, N>) -> Self {
        triangle.0
    }
}

pub fn world_vertices(geojson_path: impl AsRef<Path>) -> Vec<Vertex> {
    let geojson = parse_geojson(geojson_path);
    let mut vertices = Vec::new();
    for latlong in geojson_to_vertices(&geojson).chunks_exact(2) {
        assert_eq!(latlong.len(), 2);
        let lat = latlong[1];
        let long = latlong[0];
        let vec = latlong2xyz(lat, long);
        vertices.push(Vertex {
            position: vec.into_array(),
            normal: vec.into_array(),
        });
    }
    vertices
}

/// Converts the latitude - longitude coordinates (in degrees) into
/// xyz coordinates on a sphere with radius 1.
fn latlong2xyz<T: Float>(lat_deg: T, long_deg: T) -> Vector<T, 3> {
    let lat = lat_deg.to_radians();
    let long = long_deg.to_radians();
    let x = lat.cos() * long.sin();
    let y = -lat.sin();
    let z = lat.cos() * -long.cos();
    Vector::from_array([x, y, z])
}

fn parse_geojson(path: impl AsRef<Path>) -> GeoJson {
    let geojson_str = fs::read_to_string(path).unwrap();
    let geojson = geojson_str.parse::<GeoJson>().unwrap();
    geojson
}

/// Process top-level GeoJSON items
fn geojson_to_vertices(gj: &GeoJson) -> Vec<f32> {
    let mut vertices = Vec::new();
    match *gj {
        GeoJson::FeatureCollection(ref ctn) => {
            for feature in &ctn.features {
                if let Some(ref geom) = feature.geometry {
                    vertices.extend(match_geometry(geom));
                }
            }
        }
        GeoJson::Feature(ref feature) => {
            if let Some(ref geom) = feature.geometry {
                vertices.extend(match_geometry(geom));
            }
        }
        GeoJson::Geometry(ref geometry) => vertices.extend(match_geometry(geometry)),
    }
    vertices
}

/// Process GeoJSON geometries
fn match_geometry(geom: &Geometry) -> Vec<f32> {
    let mut vertices = Vec::new();
    match &geom.value {
        Value::Polygon(p) => {
            log::debug!("Matched a Polygon");
            vertices.extend(process_polygon_with_holes(p, 102.0));
        }
        Value::MultiPolygon(polygons) => {
            log::debug!("Matched a MultiPolygon");
            for p in polygons {
                vertices.extend(process_polygon_with_holes(p, 102.0));
            }
        }
        Value::GeometryCollection(ref gc) => {
            log::debug!("Matched a GeometryCollection");
            // GeometryCollections contain other Geometry types, and can nest
            // we deal with this by recursively processing each geometry
            for geometry in gc {
                vertices.extend(match_geometry(geometry))
            }
        }
        // Point, LineString, and their Multi– counterparts
        _ => todo!("Matched some other geometry"),
    }
    vertices
}

fn process_polygon_with_holes(polygon: &geojson::PolygonType, max_edge_length: f64) -> Vec<f32> {
    let (flat_vertices, hole_indices, dims) = earcutr::flatten(polygon);
    assert_eq!(dims, 2);

    let triangle_vertice_start_indices = earcutr::earcut(&flat_vertices, &hole_indices, dims);
    let mut output = Vec::new();
    for &[i, j, k] in triangle_vertice_start_indices.as_chunks::<3>().0 {
        let v0 = Vector::<f64, 2>::try_from(&flat_vertices[i * dims..i * dims + 2]).unwrap();
        let v1 = Vector::<f64, 2>::try_from(&flat_vertices[j * dims..j * dims + 2]).unwrap();
        let v2 = Vector::<f64, 2>::try_from(&flat_vertices[k * dims..k * dims + 2]).unwrap();
        let triangle = Triangle([v0, v1, v2]);
        let mut triangles = Vec::new();
        subdivide_sphere_face(triangle, max_edge_length, &mut triangles);
        for triangle in triangles {
            let vertices = triangle.vertices();
            for vertex in vertices {
                output.extend(vertex.as_array().iter().map(|&f| f as f32));
            }
        }
    }
    output
}

fn subdivide_sphere_face(
    triangle: Triangle<f64, 2>,
    max_edge_length: f64,
    output: &mut Vec<Triangle<f64, 2>>,
) {
    let vertices = cut_too_long_triangle_edges(triangle, max_edge_length);
    assert!(vertices.len() >= 3);
    // Base case. Triangle was small enough already
    if vertices.len() == 3 {
        output.push(triangle);
        return;
    }
    log::debug!("A triangle is too large! {:#?} {:#?}", triangle, vertices);

    // If triangle is too large. Cut up the edges so they become smaller and run
    // the resulting polygon through triangulation.
    let flat_vertices = vertices
        .iter()
        .map(|v| v.as_array().iter())
        .flatten()
        .cloned()
        .collect::<Vec<_>>();
    const DIMS: usize = 2;
    let triangle_vertice_start_indices = earcutr::earcut(&flat_vertices, &vec![], DIMS);
    for &[i, j, k] in triangle_vertice_start_indices.as_chunks::<3>().0 {
        let v0 = Vector::<f64, 2>::try_from(&flat_vertices[i * DIMS..i * DIMS + 2]).unwrap();
        let v1 = Vector::<f64, 2>::try_from(&flat_vertices[j * DIMS..j * DIMS + 2]).unwrap();
        let v2 = Vector::<f64, 2>::try_from(&flat_vertices[k * DIMS..k * DIMS + 2]).unwrap();
        let sub_triangle = Triangle([v0, v1, v2]);
        subdivide_sphere_face(sub_triangle, max_edge_length, output);
    }
}

fn cut_too_long_triangle_edges(
    triangle: Triangle<f64, 2>,
    max_edge_length: f64,
) -> Vec<Vector<f64, 2>> {
    let [v0, v1, v2] = <[Vector<f64, 2>; 3]>::from(triangle);
    let mut output = Vec::with_capacity(3);
    cut_too_long_edge(v0, v1, max_edge_length, &mut output);
    cut_too_long_edge(v1, v2, max_edge_length, &mut output);
    cut_too_long_edge(v2, v0, max_edge_length, &mut output);
    output
}

/// If the vector between `v0` and `v1` is longer than `max_edge_length` then it's divided into
/// equal sections. The return vector contains `v0` and any points in between, but not `v1`.
fn cut_too_long_edge<const N: usize>(
    v0: Vector<f64, N>,
    v1: Vector<f64, N>,
    max_edge_length: f64,
    output: &mut Vec<Vector<f64, N>>,
) {
    assert!(max_edge_length > 0.0);
    output.push(v0);
    // A vector pointing from v0 to v1
    let edge = v1 - v0;
    let num_sections = (edge.length() / max_edge_length).ceil();
    if num_sections > 1.0 {
        output.reserve(num_sections as usize);
        let section = edge / num_sections;
        for i in 1..num_sections as usize {
            output.push(v0 + (i as f64 * section));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn subdivide_sphere_face_bug() {
        let triangle: Triangle<f64, 2> = Triangle::from([
            Vector::from_array([72.07569420700005, 66.235825914]),
            Vector::from_array([123.27637780050011, 65.45589834200004]),
            Vector::from_array([174.47706139400017, 64.67597077000009]),
        ]);
        let mut output = Vec::new();
        subdivide_sphere_face(triangle, 102.0, &mut output);
        eprintln!("Output triangles: {:#?}", output);
    }

    #[test]
    fn subdivide_sphere_face_small_enough() {
        //   v2
        //   |\
        // 1 |  \ sqrt(2)
        //   |____\
        //   v0 1 v1
        let v0 = Vector::from_array([0.0, 0.0]);
        let v1 = Vector::from_array([1.0, 0.0]);
        let v2 = Vector::from_array([0.0, 1.0]);

        let mut output = Vec::new();
        subdivide_sphere_face(Triangle::from([v0, v1, v2]), 1.0, &mut output);

        // Hypotenuse is too long.
        let hypotenuse_middle_point = Vector::from_array([0.5, 0.5]);
        assert_eq!(
            output,
            &[
                Triangle::from([hypotenuse_middle_point, v2, v0],),
                Triangle::from([v0, v1, hypotenuse_middle_point],),
            ]
        );
    }

    #[test]
    fn cut_too_long_edges_same_point() {
        let v0 = Vector::from_array([1.0, 1.0]);
        let v1 = v0;

        let mut output = Vec::new();
        cut_too_long_edge(v0, v1, 0.5, &mut output);
        assert_eq!(output, &[v0]);
    }

    #[test]
    fn cut_too_long_edges_not_too_long() {
        let v0 = Vector::from_array([1.0, 1.0]);
        let v1 = Vector::from_array([0.5, 0.5]);

        let mut output = Vec::new();
        cut_too_long_edge(v0, v1, 0.71, &mut output);
        assert_eq!(output, &[v0]);
    }

    #[test]
    fn cut_too_long_edges_exactly_max_len() {
        let v0 = Vector::from_array([1.1, 0.0]);
        let v1 = Vector::from_array([0.0, 0.0]);

        let mut output = Vec::new();
        cut_too_long_edge(v0, v1, 1.1, &mut output);
        assert_eq!(output, &[v0]);
    }

    #[test]
    fn cut_too_long_edges_just_too_long() {
        let v0 = Vector::from_array([5.0, 0.01]);
        let v1 = Vector::from_array([0.0, 0.0]);
        let middle = Vector::from_array([2.5, 0.005]);

        let mut output = Vec::new();
        cut_too_long_edge(v0, v1, 5.0, &mut output);
        assert_eq!(output, &[v0, middle]);
    }

    #[test]
    fn cut_too_long_edges_much_too_long() {
        let v0 = Vector::from_array([0.0, 0.0]);
        let v1 = Vector::from_array([3.0, 0.0]);
        let section1 = Vector::from_array([1.0, 0.0]);
        let section2 = Vector::from_array([2.0, 0.0]);

        let mut output = Vec::new();
        cut_too_long_edge(v0, v1, 1.0, &mut output);
        assert_eq!(output, &[v0, section1, section2]);
    }
}
