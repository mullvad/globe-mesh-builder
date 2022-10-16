#![feature(array_chunks)]
#![feature(slice_as_chunks)]

use clap::Parser;
use std::collections::hash_map::Entry;
use std::collections::HashMap;
use std::f32::consts::PI;
use std::path::{Path, PathBuf};

mod geo;
mod linalg;

use linalg::{Triangle, Vector, Vertex};

const MAX_EDGE_LENGTH_KM: f32 = 250.0;

#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
    /// Path to the GeoJSON input file to process
    #[arg(short, long)]
    geojson: PathBuf,

    /// If the output should be pretty json or not
    #[arg(long)]
    pretty: bool,

    /// If too long triangle edges should be cut down into smaller sections
    /// to not take a too noticeable shortcut through the sphere.
    #[arg(long)]
    subdivide: bool,
}

/// The structur this program outputs (in JSON format).
#[derive(Debug, serde::Serialize)]
struct Output {
    /// A vector with vertice float values for OpenGL to render. each group of three
    /// floats is one vertice. So `(positions[x], positions[x+1], positions[x+2])` is
    /// one vertice.
    positions: Vec<f32>,
    /// The indexes of the vertices that makes up the triangles of the earth.
    ///
    /// # Example
    ///
    /// If `[indices[0], indices[1], indices[2]] == [7, 2, 0]`,
    /// then the vertices for the first triangle to draw is:
    /// `vertice0 = [positions[7*3], positions[7*3+1], positions[7*3+2]]
    /// `vertice1 = [positions[2*3], positions[2*3+1], positions[2*3+2]]
    /// `vertice2 = [positions[0*3], positions[0*3+1], positions[0*3+2]]
    ///
    /// The `*3` part comes from the fact that the `indices` says which vertice,
    /// to pull from `positions`, and each vertice occupies three elements in `positions`.
    indices: Vec<u32>,
}

fn main() {
    env_logger::init();
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
                output.positions.extend(vertex.to_vector().into_array());
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

pub fn world_vertices(geojson_path: impl AsRef<Path>, subdivide: bool) -> Vec<Vertex> {
    let world_triangles = geo::triangulate_geojson(geojson_path);
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
        vertices.extend(triangle_3d.to_vertices().map(Vertex::from));
    }
    log::info!("Converted triangles into {} vertices", vertices.len());
    vertices
}

/// Takes a triangle as input, finds the longest side of it and if that is longer
/// than MAX_EDGE_LENGTH_KM in haversine distance, cut it into two, and recurse.
fn subdivide_triangle(triangle: Triangle<f32, 2>) -> Vec<Triangle<f32, 2>> {
    let distance = |v0: Vector<f32, 2>, v1: Vector<f32, 2>| {
        let [long1, lat1] = v0.into_array();
        let [long2, lat2] = v1.into_array();
        haversine_dist(lat1, long1, lat2, long2)
    };
    let [v0, v1, v2] = triangle.to_vertices();
    let d01 = distance(v0, v1);
    let d12 = distance(v1, v2);
    let d20 = distance(v2, v0);
    let mut output = Vec::with_capacity(2);

    if d01 > MAX_EDGE_LENGTH_KM && d01 >= d12 && d01 >= d20 {
        // The side between v0 and v1 is the longest, and it's too long
        let [new_triangle1, new_triangle2] = split_triangle(Triangle::from([v2, v0, v1]));
        output.extend(subdivide_triangle(new_triangle1));
        output.extend(subdivide_triangle(new_triangle2));
    } else if d12 > MAX_EDGE_LENGTH_KM && d12 >= d20 && d12 > d01 {
        // The side between v1 and v2 is the longest, and it's too long
        let [new_triangle1, new_triangle2] = split_triangle(Triangle::from([v0, v1, v2]));
        output.extend(subdivide_triangle(new_triangle1));
        output.extend(subdivide_triangle(new_triangle2));
    } else if d20 > MAX_EDGE_LENGTH_KM && d20 >= d01 && d20 >= d12 {
        // The side between v2 and v0 is the longest, and it's too long
        let [new_triangle1, new_triangle2] = split_triangle(Triangle::from([v1, v2, v0]));
        output.extend(subdivide_triangle(new_triangle1));
        output.extend(subdivide_triangle(new_triangle2));
    } else {
        // No side is too long
        output.push(triangle);
    }
    output
}

/// Implemented as per https://en.wikipedia.org/wiki/Haversine_formula
/// and https://rosettacode.org/wiki/Haversine_formula#Rust
/// Takes input as radians, outputs kilometers.
fn haversine_dist(lat: f32, lon: f32, other_lat: f32, other_lon: f32) -> f32 {
    const RAIDUS_OF_EARTH: f32 = 6372.8;

    let d_lat = lat - other_lat;
    let d_lon = lon - other_lon;
    // Computing the haversine between two points
    let haversine =
        (d_lat / 2.0).sin().powi(2) + (d_lon / 2.0).sin().powi(2) * lat.cos() * other_lat.cos();

    // using the haversine to compute the distance between two points
    haversine.sqrt().asin() * 2.0 * RAIDUS_OF_EARTH
}

/// Splits a triangle into two. The edge that is cut into two is the one between v1 and v2.
fn split_triangle(triangle: Triangle<f32, 2>) -> [Triangle<f32, 2>; 2] {
    let [v0, v1, v2] = triangle.to_vertices();
    let v1v2_midpoint = (v1 + v2) / 2.0;
    [
        Triangle::from([v0, v1, v1v2_midpoint]),
        Triangle::from([v0, v1v2_midpoint, v2]),
    ]
}

/// Maps a 2D triangle with latitude and longitude coordinates in degrees onto
/// a sphere with radius 1
fn latlong_triangle_to_sphere(triangle: Triangle<f32, 2>) -> Triangle<f32, 3> {
    let [v0, v1, v2] = triangle.to_vertices();
    let v0 = latlong2xyz(v0);
    let v1 = latlong2xyz(v1);
    let v2 = latlong2xyz(v2);
    Triangle::from([v0, v1, v2])
}

fn latlong2xyz(longlat: Vector<f32, 2>) -> Vector<f32, 3> {
    let [long, lat] = longlat.into_array();
    // Polar angle. 0 <= φ <= PI = colatitude in geography
    let phi = (PI / 2.0) - lat;
    // Azimuthal angle = longitude. 0 <= θ <= 2*PI
    let theta = long + PI;
    let x = -(phi.sin() * theta.cos());
    let z = phi.sin() * theta.sin();
    let y = phi.cos();
    Vector::from_array([x, y, z])
}
