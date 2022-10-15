#![feature(array_chunks)]
#![feature(slice_as_chunks)]

use clap::Parser;
use geojson::{GeoJson, Geometry, Value};
use num_traits::float::Float;
use std::collections::hash_map::Entry;
use std::collections::HashMap;
use std::convert::TryFrom;
use std::f64::consts::PI;
use std::hash::Hash;
use std::path::{Path, PathBuf};
use std::{fs, thread};
use total_float_wrap::TotalF64;
use vecmat::Vector;

const MAX_EDGE_LENGTH: f64 = 0.017453071 * 100.0;

#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
    /// Path to the GeoJSON input file to process
    #[arg(short, long)]
    geojson: PathBuf,

    /// If the output should be pretty json or not
    #[arg(long)]
    pretty: bool,

    /// If the sphere mapped triangles should be subdivided or not
    #[arg(long)]
    subdivide: bool,
}

fn main() {
    env_logger::init();
    thread::Builder::new()
        .stack_size(10 * 1024 * 1024)
        .spawn(run)
        .unwrap()
        .join()
        .unwrap();
}

#[inline(never)]
fn run() {
    let args = Args::parse();
    let mesh: Vec<Vertex> = world_vertices(args.geojson, args.subdivide);
    let mut seen_vertices: HashMap<Vertex, u32> = HashMap::new();
    let mut output = Output {
        positions: Vec::new(),
        indices: Vec::new(),
    };

    let num_2d_vertices = mesh.len();
    log::info!("Total vertices: {}", num_2d_vertices);
    let mut next_index: u32 = 0;
    for vertex in mesh {
        match seen_vertices.entry(vertex) {
            Entry::Occupied(entry) => {
                output.indices.push(*entry.get());
            }
            Entry::Vacant(entry) => {
                entry.insert(next_index);
                output.indices.push(next_index);
                output
                    .positions
                    .extend(vertex.0.into_array().map(|f| f as f32));
                next_index = next_index.checked_add(1).unwrap();
            }
        }
    }

    eprintln!(
        "Vertices after removing duplicates: {}",
        seen_vertices.len()
    );
    eprintln!(
        "Ratio: {}",
        seen_vertices.len() as f32 / num_2d_vertices as f32
    );

    let stdout = std::io::stdout().lock();
    if args.pretty {
        serde_json::to_writer_pretty(stdout, &output).unwrap();
    } else {
        serde_json::to_writer(stdout, &output).unwrap();
    }
}

#[derive(Debug, serde::Serialize)]
struct Output {
    positions: Vec<f32>,
    indices: Vec<u32>,
}

/// A struct representing a single vertex in 3D space with a normal.
///
/// The `repr(C)` attribute is required to ensure that the memory layout is
/// what we expect. Without it, no specific layout is guaranteed.
#[derive(Copy, Clone, Debug)]
#[repr(C)]
pub struct Vertex(Vector<f64, 3>);

impl PartialEq for Vertex {
    fn eq(&self, other: &Self) -> bool {
        (self.0 - other.0).length() <= f64::EPSILON
    }
}

impl Eq for Vertex {}

impl Hash for Vertex {
    fn hash<H: std::hash::Hasher>(&self, state: &mut H) {
        let [v0, v1, v2] = self.0.into_array();
        TotalF64(v0).hash(state);
        TotalF64(v1).hash(state);
        TotalF64(v2).hash(state);
    }
}

#[derive(Copy, Clone, Debug)]
pub struct Triangle<T: Float, const N: usize>([Vector<T, N>; 3]);

impl<T: Float, const N: usize> Triangle<T, N> {
    pub fn vertices(self) -> [Vector<T, N>; 3] {
        self.0
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

pub fn world_vertices(geojson_path: impl AsRef<Path>, subdivide: bool) -> Vec<Vertex> {
    let geojson = parse_geojson(geojson_path);
    let world_triangles = geojson_to_triangles(&geojson);
    let num_2d_triangles = world_triangles.len();
    log::info!(
        "Parsed and earcutrd GeoJson has {} triangles",
        num_2d_triangles
    );

    let mut world_triangles_sphere = Vec::with_capacity(world_triangles.len());
    for triangle_2d in world_triangles {
        if subdivide {
            for subdivided_triangle in subdivide_triangle(triangle_2d) {
                world_triangles_sphere.push(latlong_triangle_to_sphere(subdivided_triangle));
            }
        } else {
            world_triangles_sphere.push(latlong_triangle_to_sphere(triangle_2d));
        }
    }
    log::info!(
        "Mapped {} 2D triangles onto {} 3D triangles on a sphere",
        num_2d_triangles,
        world_triangles_sphere.len()
    );
    let mut vertices = Vec::with_capacity(world_triangles_sphere.len() * 3);
    for triangle_3d in world_triangles_sphere {
        vertices.extend(triangle_3d.vertices().map(Vertex));
    }
    log::info!("Converted triangles into {} vertices", vertices.len());
    vertices
}

fn subdivide_triangle(triangle: Triangle<f64, 2>) -> Vec<Triangle<f64, 2>> {
    let [v0, v1, v2] = triangle.vertices();
    let e01 = v1 - v0;
    let e12 = v2 - v1;
    let e20 = v0 - v2;
    let mut output = Vec::new();
    if e01.length() > MAX_EDGE_LENGTH
        && e01.length() >= e12.length()
        && e01.length() >= e20.length()
    {
        let [new_triangle1, new_triangle2] = split_triangle(Triangle::from([v2, v0, v1]));
        output.extend(subdivide_triangle(new_triangle1));
        output.extend(subdivide_triangle(new_triangle2));
    } else if e12.length() > MAX_EDGE_LENGTH
        && e12.length() >= e20.length()
        && e12.length() > e01.length()
    {
        let [new_triangle1, new_triangle2] = split_triangle(Triangle::from([v0, v1, v2]));
        output.extend(subdivide_triangle(new_triangle1));
        output.extend(subdivide_triangle(new_triangle2));
    } else if e20.length() > MAX_EDGE_LENGTH
        && e20.length() >= e01.length()
        && e20.length() >= e12.length()
    {
        let [new_triangle1, new_triangle2] = split_triangle(Triangle::from([v1, v2, v0]));
        output.extend(subdivide_triangle(new_triangle1));
        output.extend(subdivide_triangle(new_triangle2));
    } else {
        // No side is too long
        assert!(e01.length() <= MAX_EDGE_LENGTH);
        assert!(e12.length() <= MAX_EDGE_LENGTH);
        assert!(e20.length() <= MAX_EDGE_LENGTH);
        output.push(triangle);
    }
    assert!(!output.is_empty());
    output
}

/// Splits a triangle into two. The edge that is cut into two is the one between v1 and v2.
fn split_triangle(triangle: Triangle<f64, 2>) -> [Triangle<f64, 2>; 2] {
    let [v0, v1, v2] = triangle.vertices();
    let v1v2_midpoint = (v1 + v2) / 2.0;
    [
        Triangle::from([v0, v1, v1v2_midpoint]),
        Triangle::from([v0, v1v2_midpoint, v2]),
    ]
}

/// Maps a 2D triangle with latitude and longitude coordinates in degrees onto
/// a sphere with radius 1
fn latlong_triangle_to_sphere(triangle: Triangle<f64, 2>) -> Triangle<f64, 3> {
    let [v0, v1, v2] = triangle.vertices();
    let v0 = latlong2xyz(v0);
    let v1 = latlong2xyz(v1);
    let v2 = latlong2xyz(v2);
    Triangle::from([v0, v1, v2])
}

/// Converts the latitude - longitude coordinates (in degrees) into
/// xyz coordinates on a sphere with radius 1.
// fn latlong2xyz<T: Float>(longlat: Vector<T, 2>) -> Vector<T, 3> {
//     let [long, lat] = longlat.into_array().map(T::to_radians);
//     let x = lat.cos() * long.sin();
//     let y = -lat.sin();
//     let z = lat.cos() * -long.cos();
//     Vector::from_array([x, y, z])
// }

fn latlong2xyz(longlat: Vector<f64, 2>) -> Vector<f64, 3> {
    let [long, lat] = longlat.into_array().map(f64::to_radians);
    // Polar angle. 0 <= φ <= PI = colatitude in geography
    let phi = (PI / 2.0) - lat;
    // Azimuthal angle = longitude. 0 <= θ <= 2*PI
    let theta = long + PI;
    let x = -(phi.sin() * theta.cos());
    let z = phi.sin() * theta.sin();
    let y = phi.cos();
    Vector::from_array([x, y, z])
}

#[test]
fn latlong2xyz2latlong() {
    check(Vector::from_array([0.0, 0.0]));
    check(Vector::from_array([90.0, 0.0]));

    fn check(longlat: Vector<f64, 2>) {
        let xyz = latlong2xyz(longlat);
        let output_longlat = dbg!(xyz2latlong(xyz));
        assert!((longlat - output_longlat).length() <= f64::EPSILON * 20.0);
    }
}

/// A flat world
// fn latlong2xyz(longlat: Vector<f64, 2>) -> Vector<f64, 3> {
//     let [long, lat] = longlat.into_array().map(f64::to_radians);
//     let x = long / std::f64::consts::PI;
//     let y = lat / std::f64::consts::PI * 2.0;
//     let z = 0.0;
//     Vector::from_array([x, y, z])
// }

fn parse_geojson(path: impl AsRef<Path>) -> GeoJson {
    let geojson_str = fs::read_to_string(path).unwrap();
    geojson_str.parse::<GeoJson>().unwrap()
}

/// Process top-level GeoJSON items
fn geojson_to_triangles(gj: &GeoJson) -> Vec<Triangle<f64, 2>> {
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
fn match_geometry(geom: &Geometry) -> Vec<Triangle<f64, 2>> {
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
                vertices.extend(match_geometry(geometry))
            }
        }
        // Point, LineString, and their Multi– counterparts
        _ => todo!("Matched some other geometry"),
    }
    vertices
}

/// Takes a 2D polygon, performs earcutr and returns the 2D triangles
fn process_polygon_with_holes(polygon: &geojson::PolygonType) -> Vec<Triangle<f64, 2>> {
    let (flat_vertices, hole_indices, dims) = earcutr::flatten(polygon);
    assert_eq!(dims, 2);

    let triangle_vertice_start_indices = earcutr::earcut(&flat_vertices, &hole_indices, dims);
    let mut output = Vec::with_capacity(triangle_vertice_start_indices.len() / 3);
    for &[i, j, k] in triangle_vertice_start_indices.as_chunks::<3>().0 {
        let v0 = Vector::<f64, 2>::try_from(&flat_vertices[i * dims..i * dims + 2]).unwrap();
        let v1 = Vector::<f64, 2>::try_from(&flat_vertices[j * dims..j * dims + 2]).unwrap();
        let v2 = Vector::<f64, 2>::try_from(&flat_vertices[k * dims..k * dims + 2]).unwrap();
        let triangle = Triangle([v0, v1, v2]);
        output.push(triangle);
    }
    output
}

#[cfg(test)]
mod tests {
    use super::*;

    // #[test]
    // fn subdivide_sphere_face_small_enough() {
    //     //   v2
    //     //   |\
    //     // 1 |  \ sqrt(2)
    //     //   |____\
    //     //   v0 1 v1
    //     let v0 = Vector::from_array([0.0, 0.0]);
    //     let v1 = Vector::from_array([1.0, 0.0]);
    //     let v2 = Vector::from_array([0.0, 1.0]);

    //     let triangles = subdivide_triangle(Triangle::from([v0, v1, v2]));

    //     // Hypotenuse is too long.
    //     let hypotenuse_middle_point = Vector::from_array([0.5, 0.5]);
    //     assert_eq!(
    //         output,
    //         &[
    //             Triangle::from([hypotenuse_middle_point, v2, v0],),
    //             Triangle::from([v0, v1, hypotenuse_middle_point],),
    //         ]
    //     );
    // }

    #[test]
    fn split_triangle_lol() {
        let v0 = Vector::from_array([0.0, 0.0, 0.0]);
        let v1 = Vector::from_array([-1.0, -1.0, 0.4]);
        let v2 = Vector::from_array([1.0, -1.0, 0.5]);
        let [t1, t2] = split_triangle(Triangle::from([v0, v1, v2]));
        assert_eq!(
            t1.vertices(),
            [v0, v1, Vector::from_array([0.0, -1.0, 0.45])]
        );
    }

    // #[test]
    // fn cut_too_long_edges_same_point() {
    //     let v0 = Vector::from_array([1.0, 1.0]);
    //     let v1 = v0;

    //     let mut output = Vec::new();
    //     cut_too_long_edge(v0, v1, 0.5, &mut output);
    //     assert_eq!(output, &[v0]);
    // }

    // #[test]
    // fn cut_too_long_edges_not_too_long() {
    //     let v0 = Vector::from_array([1.0, 1.0]);
    //     let v1 = Vector::from_array([0.5, 0.5]);

    //     let mut output = Vec::new();
    //     cut_too_long_edge(v0, v1, 0.71, &mut output);
    //     assert_eq!(output, &[v0]);
    // }

    // #[test]
    // fn cut_too_long_edges_exactly_max_len() {
    //     let v0 = Vector::from_array([1.1, 0.0]);
    //     let v1 = Vector::from_array([0.0, 0.0]);

    //     let mut output = Vec::new();
    //     cut_too_long_edge(v0, v1, 1.1, &mut output);
    //     assert_eq!(output, &[v0]);
    // }

    // #[test]
    // fn cut_too_long_edges_just_too_long() {
    //     let v0 = Vector::from_array([5.0, 0.01]);
    //     let v1 = Vector::from_array([0.0, 0.0]);
    //     let middle = Vector::from_array([2.5, 0.005]);

    //     let mut output = Vec::new();
    //     cut_too_long_edge(v0, v1, 5.0, &mut output);
    //     assert_eq!(output, &[v0, middle]);
    // }

    // #[test]
    // fn cut_too_long_edges_much_too_long() {
    //     let v0 = Vector::from_array([0.0, 0.0]);
    //     let v1 = Vector::from_array([3.0, 0.0]);
    //     let section1 = Vector::from_array([1.0, 0.0]);
    //     let section2 = Vector::from_array([2.0, 0.0]);

    //     let mut output = Vec::new();
    //     cut_too_long_edge(v0, v1, 1.0, &mut output);
    //     assert_eq!(output, &[v0, section1, section2]);
    // }
}
