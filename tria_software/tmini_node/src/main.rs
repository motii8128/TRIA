// use std::println;

// use rust_tmini_driver::TMini;

// use hiroz::{
//     Builder,
//     Result,
//     context::ZContextBuilder
// };

// use hiroz_msgs::{
//     sensor_msgs::PointCloud,
//     geometry_msgs::Point32
// };


// #[tokio::main]
// async fn main() -> Result<()> {

//     let format = hiroz_protocol::KeyExprFormat::RmwZenoh;

//     let ctx = ZContextBuilder::default().with_mode("peer").keyexpr_format(format).build()?;
//     let node = ctx.create_node("tmini_node").with_type_description_service().build()?;
//     let scan_publisher = node.create_pub::<PointCloud>("scan").build()?;

//     // let tmini = TMini::new("/dev/ttyUSB0", 10.0, false, false)?;

//     println!("Starting scan loop. Press Ctrl-C to exit...");

//     loop {
//         // match tmini.get_scan() {
//         //     Ok(scan) => {
//         //         let mut point_cloud = PointCloud::default();
//         //         point_cloud.header.frame_id = "map".to_string();
                
//         //         for i in 0..400 {
//         //             let x = scan.range[i] * scan.angle[i].cos();
//         //             let y = scan.range[i] * scan.angle[i].sin();
//         //             let point = Point32 { x, y, z: 0.0 };
//         //             point_cloud.points.push(point);
//         //         }
                
//         //         scan_publisher.async_publish(&point_cloud).await?;
//         //     },
//         //     Err(e) => {
//         //         eprintln!("Error occurred: {}", e);
//         //     }
//         // }

//         println!("Publish Test");

//         let mut p = PointCloud::default();
//         p.header.frame_id = "s".to_string();

//         scan_publisher.async_publish(&p).await?;
        
//         let _ = tokio::time::sleep(std::time::Duration::from_millis(100)).await;
//     }
// }

use std::time::Duration;
use hiroz::{Builder, Result, context::ZContextBuilder};
use hiroz_msgs::std_msgs::String as RosString;

#[tokio::main]
async fn main() -> Result<()> {

    zenoh::init_log_from_env_or("error");

    // Initialize hiroz context (connects to router on localhost:7447)
    let ctx = ZContextBuilder::default()
        .with_connect_endpoints(["tcp/127.0.0.1:7447"])
        .build()?;

    // Create a ROS 2 node
    let node = ctx.create_node("my_talker").build()?;

    // Create a publisher for the /chatter topic
    let pub_handle = node.create_pub::<RosString>("/chatter").build()?;

    // Publish messages every second
    let mut count = 0;
    loop {
        let msg = RosString {
            data: format!("Hello from hiroz #{}", count),
        };
        println!("Publishing: {}", msg.data);
        pub_handle.async_publish(&msg).await?;
        count += 1;
        tokio::time::sleep(Duration::from_secs(1)).await;
    }
}