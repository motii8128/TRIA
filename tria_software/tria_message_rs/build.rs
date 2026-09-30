use std::io::Result;
fn main() -> Result<()> {
    prost_build::compile_protos(&["./proto/tria_protocol.proto"], &["./proto/"])?;
    Ok(())
}