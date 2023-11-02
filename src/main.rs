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
use std::collections::hash_map::Entry;
use std::collections::HashMap;
use std::f32::consts::PI;
use std::fs::OpenOptions;
use std::io::Write;
use std::path::{Path, PathBuf};

mod geo;
mod icosahedron;
mod linalg;

use linalg::{Triangle, Vector, Vertex};

pub const MIN_2D_POLAR_COORDINATE_ERROR: f32 = f32::EPSILON * 2.0;

#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
    /// Path(s) to the .shp input file(s) to process
    #[arg(long)]
    shp: Vec<PathBuf>,

    /// If too long triangle edges should be cut down into smaller sections
    /// to not take a too noticeable shortcut through the sphere.
    #[arg(long)]
    subdivide: bool,
}

/// The structur this program outputs (in JSON format).
#[derive(Debug)]
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

#[derive(Debug)]
struct SphereOutput {
    positions: Vec<f32>,
    indices: Vec<u32>,
}

fn main() {
    env_logger::init();
    let args = Args::parse();

    let ocean = ocean_vertices();
    write_buffer_f32("geodata/ocean_positions.bin", &ocean.positions);
    write_buffer_u32("geodata/ocean_indices.bin", &ocean.indices);

    let (triangles, contours) = world_vertices(&args.shp, args.subdivide);

    let mut seen_vertices: HashMap<Vertex, u32> = HashMap::new();
    let mut land_output = Output {
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
                land_output.triangle_indices.push(*entry.get());
            }
            Entry::Vacant(entry) => {
                entry.insert(next_index);
                land_output.triangle_indices.push(next_index);
                land_output
                    .positions
                    .extend(vertex.to_vector().into_array());
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
                        land_output.contour_indices.push(*entry.get());
                    }
                    Entry::Vacant(entry) => {
                        entry.insert(next_index);
                        land_output.contour_indices.push(next_index);
                        land_output
                            .positions
                            .extend(vertex.to_vector().into_array());
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

    write_buffer_f32("geodata/land_positions.bin", &land_output.positions);
    write_buffer_u32(
        "geodata/land_triangle_indices.bin",
        &land_output.triangle_indices,
    );
    write_buffer_u32(
        "geodata/land_contour_indices.bin",
        &land_output.contour_indices,
    );
}

fn write_buffer_f32(path: impl AsRef<Path>, data: &[f32]) {
    let mut file = OpenOptions::new()
        .write(true)
        .create(true)
        .truncate(true)
        .open(path)
        .unwrap();
    for position in data {
        file.write_all(&position.to_ne_bytes()).unwrap();
    }
    file.flush().unwrap();
}

fn write_buffer_u32(path: impl AsRef<Path>, data: &[u32]) {
    let mut file = OpenOptions::new()
        .write(true)
        .create(true)
        .truncate(true)
        .open(path)
        .unwrap();
    for position in data {
        file.write_all(&position.to_ne_bytes()).unwrap();
    }
    file.flush().unwrap();
}

fn ocean_vertices() -> SphereOutput {
    let mut seen_vertices: HashMap<Vertex, u32> = HashMap::new();
    let mut output = SphereOutput {
        positions: Vec::new(),
        indices: Vec::new(),
    };
    for vertex in icosahedron::icosahedron_vertices(4, 0.998) {
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
    output
}

pub fn world_vertices(
    world_shp_paths: &[impl AsRef<Path>],
    subdivide: bool,
) -> (Vec<Vertex>, Vec<Vec<Vertex>>) {
    let mut world_triangles = Vec::new();
    let mut world_contours = Vec::new();
    for world_shp_path in world_shp_paths {
        let (triangles, contours) = geo::read_world(world_shp_path);
        world_triangles.extend(triangles);
        world_contours.extend(contours);
    }

    let num_2d_triangles = world_triangles.len();
    log::info!(
        "Parsed and earcutrd world mesh has {} triangles",
        num_2d_triangles
    );
    log::debug!("Number of world contours: {}", world_contours.len());

    let mut world_triangles_sphere = Vec::with_capacity(world_triangles.len());
    let mut world_contours_sphere = Vec::with_capacity(world_contours.len());
    for triangle_2d in world_triangles {
        if subdivide {
            for subdivided_triangle in geo::subdivide_triangle(triangle_2d) {
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

#[cfg(test)]
mod tests {
    use super::*;
    use std::f32::consts::TAU;

    #[test]
    fn subdivide_broken_triangle() {
        // let broken_triangle = geo::Triangle::from([
        //     geo::Coordinate {
        //         lat: -1.4722126,
        //         long: 3.1415927,
        //     },
        //     geo::Coordinate {
        //         lat: -1.439808,
        //         long: -0.7991614,
        //     },
        //     geo::Coordinate {
        //         lat: -1.4566238,
        //         long: -1.0387504,
        //     },
        // ]);

        let broken_triangle = geo::Triangle::from([
            geo::Coordinate {
                lat: 1.2223023,
                long: -2.5530975,
            },
            geo::Coordinate {
                lat: 1.2225933,
                long: -2.5593796,
            },
            geo::Coordinate {
                lat: 1.2190795,
                long: -2.513102,
            },
        ]);
        geo::subdivide_triangle(broken_triangle);
    }

    #[test]
    fn rem_euclid() {
        dbg!((-PI) % TAU);
        dbg!(((-PI) - 1.0) % TAU);
        dbg!((PI) % (TAU));
        dbg!((PI + 1.0) % TAU);
        dbg!((-1.0f32) % TAU);
        dbg!((1.0f32) % TAU);
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
