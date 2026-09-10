// use hiroz::{
//     Builder,
//     Result,
//     context::ZContextBuilder
// };

// use hiroz_msgs::sensor_msgs::PointCloud;

// #[tokio::main]
// async fn main() -> Result<()> {

//     let format = hiroz_protocol::KeyExprFormat::RmwZenoh;

//     // let rec = rerun::RecordingStreamBuilder::new("lidar_visualize_node").spawn()?;

//     let ctx = ZContextBuilder::default().with_mode("peer").keyexpr_format(format).build()?;
//     let node = ctx.create_node("lidar_visualize_node").build()?;

//     let scan_subscriber = node.create_sub::<PointCloud>("scan").build()?;

//     println!("Listening on /scan...");
//     // while let Ok(msg) = scan_subscriber.async_recv().await {

//     //     println!("Received scan with {} points", msg.points.len());
        
//     //     let count = msg.points.len();

//     //     let mut points = Vec::with_capacity(count);
//     //     let mut colors = Vec::with_capacity(count);

//     //     for point in msg.points
//     //     {
//     //         points.push((point.x, point.y, point.z));
//     //         colors.push(rerun::Color::from_rgb(0, 0, 255));
//     //     }

//     //     // rec.log("scan", &rerun::Points3D::new(points).with_colors(colors).with_radii([0.01]))?;
//     // }

//     loop {
//         match scan_subscriber.async_recv().await {
//             Ok(msg) => {
//                 println!("Received scan with {} points", msg.points.len());
//                 // 処理
//             }
//             Err(e) => {
//                 // 受信待ち（Emptyなど）の場合は少し待ってループを維持
//                 // tokio::time::sleep(std::time::Duration::from_millis(10)).await;
//             }
//         }
//     }
// }

use hiroz::{Builder, Result, context::ZContextBuilder};
use hiroz_msgs::std_msgs::String as RosString;

#[tokio::main]
async fn main() -> Result<()> {

    zenoh::init_log_from_env_or("error");

    let ctx = ZContextBuilder::default()
        .with_connect_endpoints(["tcp/127.0.0.1:7447"])
        .build()?;
    let node = ctx.create_node("my_listener").build()?;
    let sub = node.create_sub::<RosString>("/chatter").build()?;

    println!("Listening on /chatter...");
    while let Ok(msg) = sub.async_recv().await {
        println!("Received: {}", msg.data);
    }
    Ok(())
}

