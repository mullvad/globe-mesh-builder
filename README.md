# Globe mesh builder

Tool that parses SHP (shapefiles) and produce OpenGL compatible vertex and index buffers for
drawing a spherical world map (globe). Used by the app to generate map data.

## Usage

1. Download land and state border data from Natural Earth and unpack them:
    ```
    mkdir earth_data; cd earth_data

    curl -L -O https://www.naturalearthdata.com/http//www.naturalearthdata.com/download/50m/cultural/ne_50m_admin_0_countries.zip
    curl -L -O https://www.naturalearthdata.com/http//www.naturalearthdata.com/download/50m/cultural/ne_50m_admin_1_states_provinces_lines.zip

    unzip ne_50m_admin_0_countries.zip
    unzip ne_50m_admin_1_states_provinces_lines.zip
    ```

1. Run this tool:
    ```
    mkdir geodata

    RUST_LOG=debug cargo run --release -- \
        --shp earth_data/ne_50m_admin_0_countries.shp \
        --shp earth_data/ne_50m_admin_1_states_provinces_lines.shp \
        --subdivide
        --out geodata/
    ```

## Output format

The output files are raw dumps of the f32 and u32 vectors (in native byte order) of the vertex
and index data. These are intended to be loaded directly into OpenGL
ARRAY_BUFFERs and ELEMENT_ARRAY_BUFFERs respectively.
