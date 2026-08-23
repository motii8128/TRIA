use std::println;

use rust_tmini_driver::TMini;

use hiroz::{
    Builder,
    Result,
    context::ZContextBuilder
};

use hiroz_msgs::{
    sensor_msgs::PointCloud,
    geometry_msgs::Point32
};


#[tokio::main]
async fn main() -> Result<()> {

    let format = hiroz_protocol::KeyExprFormat::RmwZenoh;

    let ctx = ZContextBuilder::default().with_mode("peer").keyexpr_format(format).build()?;
    let node = ctx.create_node("tmini_node").with_type_description_service().build()?;
    let scan_publisher = node.create_pub::<PointCloud>("/scan").build()?;

    let tmini = TMini::new("/dev/ttyUSB0", 10.0, false, false)?;

    println!("Starting scan loop. Press Ctrl-C to exit...");

    loop {
        match tmini.get_scan() {
            Ok(scan) => {
                let mut point_cloud = PointCloud::default();
                point_cloud.header.frame_id = "map".to_string();
                
                for i in 0..400 {
                    let x = scan.range[i] * scan.angle[i].cos();
                    let y = scan.range[i] * scan.angle[i].sin();
                    let point = Point32 { x, y, z: 0.0 };
                    point_cloud.points.push(point);
                }
                
                scan_publisher.async_publish(&point_cloud).await?;
            },
            Err(e) => {
                eprintln!("Error occurred: {}", e);
            }
        }
        
        tokio::time::sleep(tokio::time::Duration::from_millis(100)).await;
    }
}