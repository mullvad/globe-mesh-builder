#![feature(array_chunks)]
#![feature(slice_as_chunks)]
#![feature(array_windows)]

//! https://public.opendatasoft.com/explore/dataset/natural-earth-countries-1_110m/table/
//! https://vvvv.org/blog/polar-spherical-and-geographic-coordinates
//! https://developer.mozilla.org/en-US/docs/Web/API/WebGL_API/Tutorial/Creating_3D_objects_using_WebGL
//! https://www.tutorialspoint.com/webgl/webgl_interactive_cube.htm
//! https://geojson-maps.ash.ms/ <- GeoJSON source data
//! https://greggman.github.io/webgl-lint/ <- Great debugging for webgl

use clap::Parser;
use geo::Coordinate;
use std::collections::hash_map::Entry;
use std::collections::{HashMap, HashSet};
use std::f32::consts::{FRAC_PI_2, PI, TAU};
use std::io::Write;
use std::path::{Path, PathBuf};
use std::thread;

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

    /// If true, outputs a sphere (to use for ocean) instead of the geo data
    #[arg(long)]
    ocean: bool,
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
    triangle_indices: Vec<u32>,
    contour_indices: Vec<u32>,
}

#[derive(Debug, serde::Serialize)]
struct SphereOutput {
    positions: Vec<f32>,
    indices: Vec<u32>,
}

fn main() {
    env_logger::init();
    let args = Args::parse();
    thread::Builder::new()
        .stack_size(1024 * 100)
        .spawn(|| run(args))
        .unwrap()
        .join()
        .unwrap();
}

fn run(args: Args) {
    if args.ocean {
        let mut seen_vertices: HashMap<Vertex, u32> = HashMap::new();
        let mut output = SphereOutput {
            positions: Vec::new(),
            indices: Vec::new(),
        };
        for vertex in icosahedron_vertices(4, 0.999) {
            let next_index = u32::try_from(seen_vertices.len()).unwrap();
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
                }
            }
        }
        log::info!(
            "Outputting {}/{} vertices",
            seen_vertices.len(),
            output.indices.len()
        );
        let mut stdout = std::io::stdout().lock();
        write!(stdout, "const oceanData = ").unwrap();
        if args.pretty {
            serde_json::to_writer_pretty(stdout, &output).unwrap();
        } else {
            serde_json::to_writer(stdout, &output).unwrap();
        }
        return;
    }

    let (triangles, contours) = world_vertices(args.geojson, args.subdivide);

    let mut seen_vertices: HashMap<Vertex, u32> = HashMap::new();
    let mut output = Output {
        positions: Vec::new(),
        triangle_indices: Vec::new(),
        contour_indices: Vec::new(),
    };

    let num_3d_vertices = triangles.len();
    for vertex in triangles {
        let next_index = u32::try_from(seen_vertices.len()).unwrap();
        match seen_vertices.entry(vertex) {
            Entry::Occupied(entry) => {
                // This vertex is already in `output.positions`,
                // just push the index
                output.triangle_indices.push(*entry.get());
            }
            Entry::Vacant(entry) => {
                entry.insert(next_index);
                output.triangle_indices.push(next_index);
                output.positions.extend(vertex.to_vector().into_array());
            }
        }
    }
    for contour in contours {
        for line_vertices in contour.array_windows::<2>() {
            for &vertex in line_vertices {
                let next_index = u32::try_from(seen_vertices.len()).unwrap();
                match seen_vertices.entry(vertex) {
                    Entry::Occupied(entry) => {
                        // This vertex is already in `output.positions`,
                        // just push the index
                        output.contour_indices.push(*entry.get());
                    }
                    Entry::Vacant(entry) => {
                        entry.insert(next_index);
                        output.contour_indices.push(next_index);
                        output.positions.extend(vertex.to_vector().into_array());
                    }
                }
            }
        }
    }

    log::info!(
        "Vertices after removing duplicates: {}. {:.1}% of original vertices",
        seen_vertices.len(),
        seen_vertices.len() as f32 / num_3d_vertices as f32 * 100.0
    );

    let mut stdout = std::io::stdout().lock();
    write!(stdout, "const globeData = ").unwrap();
    if args.pretty {
        serde_json::to_writer_pretty(stdout, &output).unwrap();
    } else {
        serde_json::to_writer(stdout, &output).unwrap();
    }
}

pub fn world_vertices(
    geojson_path: impl AsRef<Path>,
    subdivide: bool,
) -> (Vec<Vertex>, Vec<Vec<Vertex>>) {
    let (world_triangles, world_contours) = geo::read_world(geojson_path);
    // let (world_triangles, world_contours) = geo::triangulate_geojson(geojson_path);
    let num_2d_triangles = world_triangles.len();
    log::info!(
        "Parsed and earcutrd GeoJson has {} triangles",
        num_2d_triangles
    );
    log::debug!("Number of world contours: {}", world_contours.len());

    let mut world_triangles_sphere = Vec::with_capacity(world_triangles.len());
    let mut world_contours_sphere = Vec::with_capacity(world_contours.len());
    for triangle_2d in world_triangles {
        if subdivide {
            log::debug!("LOLOLOL Subdividing {:?}", triangle_2d);
            for subdivided_triangle in geo::subdivide_triangle(triangle_2d, &mut HashSet::new()) {
                world_triangles_sphere.push(geo_triangle_to_sphere(subdivided_triangle));
            }
            log::debug!("");
        } else {
            world_triangles_sphere.push(geo_triangle_to_sphere(triangle_2d));
        }
    }
    for contour in world_contours {
        world_contours_sphere.push(contour.into_iter().map(|c| latlong2xyz(c)).collect());
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
    (vertices, world_contours_sphere)
}

/// Maps a 2D geo triangle with latitude and longitude coordinates in radians onto
/// a sphere with radius 1
fn geo_triangle_to_sphere(triangle: geo::Triangle) -> Triangle {
    Triangle::from(triangle.to_coordinates().map(latlong2xyz))
}

/// Maps a geographical coordinate represented in radians onto a sphere
/// with radius 1.0, and returns the 3D vector
fn latlong2xyz(c: geo::Coordinate) -> Vertex {
    // Polar angle. 0 <= φ <= PI = colatitude in geography
    let phi = (PI / 2.0) - c.lat;
    // Azimuthal angle = longitude. -PI <= θ <= PI
    let theta = c.long;

    let x = phi.sin() * theta.sin();
    let y = phi.cos();
    let z = phi.sin() * theta.cos();
    Vertex::from(Vector::from_array([x, y, z]))
}

pub fn icosahedron_vertices(subdivide_times: u8, scale: f32) -> Vec<Vertex> {
    let mut triangles = icosahedron_faces();
    for _ in 0..subdivide_times {
        let old_triangles = core::mem::replace(&mut triangles, Vec::new());
        for triangle in old_triangles {
            triangles.extend(&subdivide_sphere_face(triangle));
        }
    }

    let mut vertices = Vec::with_capacity(triangles.len() * 3);
    for triangle in triangles {
        vertices.extend(triangle.to_vertices().map(|v| v * scale));
    }
    vertices
}

fn icosahedron_faces() -> Vec<Triangle> {
    let latlong2xyz = |lat: f32, long: f32| latlong2xyz(Coordinate { lat, long });

    let mut triangles = Vec::with_capacity(20);

    let upper_ring_lat = 0.5f32.atan();
    let lower_ring_lat = -upper_ring_lat;
    for i in 0..5 {
        let upper_long1 = i as f32 / 5.0 * TAU;
        let upper_long2 = (i + 1) as f32 / 5.0 * TAU;
        let lower_long1 = (i as f32 + 0.5) / 5.0 * TAU;
        let lower_long2 = (i as f32 + 1.5) / 5.0 * TAU;
        let top_triangle = Triangle::from([
            latlong2xyz(FRAC_PI_2, 0.),
            latlong2xyz(upper_ring_lat, upper_long1),
            latlong2xyz(upper_ring_lat, upper_long2),
        ]);
        let middle_triangle1 = Triangle::from([
            latlong2xyz(upper_ring_lat, upper_long1),
            latlong2xyz(lower_ring_lat, lower_long1),
            latlong2xyz(upper_ring_lat, upper_long2),
        ]);
        let middle_triangle2 = Triangle::from([
            latlong2xyz(upper_ring_lat, upper_long2),
            latlong2xyz(lower_ring_lat, lower_long1),
            latlong2xyz(lower_ring_lat, lower_long2),
        ]);
        let bottom_triangle = Triangle::from([
            latlong2xyz(lower_ring_lat, lower_long1),
            latlong2xyz(-FRAC_PI_2, 0.),
            latlong2xyz(lower_ring_lat, lower_long2),
        ]);
        triangles.push(top_triangle);
        triangles.push(middle_triangle1);
        triangles.push(middle_triangle2);
        triangles.push(bottom_triangle);
    }
    triangles
}

fn subdivide_sphere_face(triangle: Triangle) -> [Triangle; 4] {
    let [v0, v1, v2] = triangle.to_vertices().map(|v| v.to_vector());
    let v3 = Vertex::from((0.5 * (v0 + v1)).normalize());
    let v4 = Vertex::from((0.5 * (v1 + v2)).normalize());
    let v5 = Vertex::from((0.5 * (v2 + v0)).normalize());
    [
        Triangle::from([v0.into(), v3, v5]),
        Triangle::from([v3, v1.into(), v4]),
        Triangle::from([v4, v2.into(), v5]),
        Triangle::from([v3, v4, v5]),
    ]
}

#[cfg(test)]
mod tests {
    use std::collections::HashSet;

    use super::*;
    
    #[test]
    fn subdivide_broken_triangle() {
        let broken_triangle = geo::Triangle::from([
            geo::Coordinate { lat: -1.4722126, long: 3.1415927 },
            geo::Coordinate { lat: -1.439808, long: -0.7991614 },
            geo::Coordinate { lat: -1.4566238, long: -1.0387504 }
        ]);

        geo::subdivide_triangle(broken_triangle, &mut HashSet::new());
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

