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

    let num_3d_vertices = mesh.len();
    let mut next_index: u32 = 0;
    for vertex in mesh {
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

    log::info!(
        "Vertices after removing duplicates: {}. {:.1}% of original vertices",
        seen_vertices.len(),
        seen_vertices.len() as f32 / num_3d_vertices as f32 * 100.0
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
            for subdivided_triangle in geo::subdivide_triangle(triangle_2d) {
                world_triangles_sphere.push(geo_triangle_to_sphere(subdivided_triangle));
            }
        } else {
            world_triangles_sphere.push(geo_triangle_to_sphere(triangle_2d));
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

/// Maps a 2D triangle with latitude and longitude coordinates in radians onto
/// a sphere with radius 1
fn geo_triangle_to_sphere(triangle: geo::Triangle) -> Triangle {
    let [c0, c1, c2] = triangle.to_coordinates();
    let v0 = latlong2xyz(c0);
    let v1 = latlong2xyz(c1);
    let v2 = latlong2xyz(c2);
    Triangle::from([v0, v1, v2])
}

/// Maps a geographical coordinate represented in radians onto a sphere
/// with radius 1.0, and returns the 3D vector
fn latlong2xyz(c: geo::Coordinate) -> Vertex {
    // Polar angle. 0 <= φ <= PI = colatitude in geography
    let phi = (PI / 2.0) - c.lat;
    // Azimuthal angle = longitude. 0 <= θ <= 2*PI
    let theta = c.long + PI;
    let x = -(phi.sin() * theta.cos());
    let z = phi.sin() * theta.sin();
    let y = phi.cos();
    Vertex::from(Vector::from_array([x, y, z]))
}
