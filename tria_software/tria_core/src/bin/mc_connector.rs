use tria_message_rs::tria::{CommandPacket, SensorPacket, MachineSensors, MotorSensor};
use tria_message_rs::Message;

use tria_common_utils::robomaster::RoboMaster;

use zenohpb::Node;

use tokio::net::UdpSocket;

#[tokio::main]
async fn main() ->zenoh::Result<()>{
    let node = Node::new("micro_controller_connector", zenoh::config::Config::default()).await?;

    let wheel_cmd_subscriber = node.create_subscription::<CommandPacket>("cmd/wheel").await?;
    let machine_cmd_subscriber = node.create_subscription::<CommandPacket>("cmd/machine").await?;
    let machine_sensor_publisher = node.create_publisher::<MachineSensors>("sensor/machine").await?;

    let udp = UdpSocket::bind("192.168.11.4:64201").await?;

    let mut horizontal = RoboMaster::new();
    let mut vertical = RoboMaster::new();
    let mut hand = RoboMaster::new();

    let mut machine_sensors = MachineSensors::default();

    loop {
        let mut wheel_cmd = wheel_cmd_subscriber.subscribe().await?;
        let machine_cmd = machine_cmd_subscriber.subscribe().await?;

        wheel_cmd.hand_motor = machine_cmd.hand_motor;
        wheel_cmd.horizontal_motor = machine_cmd.horizontal_motor;
        wheel_cmd.vertical_motor = machine_cmd.vertical_motor;

        let mut send_buf = Vec::new();
        let _ = wheel_cmd.encode(&mut send_buf);

        let _ = udp.send_to(&send_buf, "192.168.11.2:64201");

        let mut buffer = [0_u8;256];

        match udp.recv(&mut buffer).await
        {
            Ok(size)=>{
                let data = &buffer[..size];
                let sensor_packet = SensorPacket::decode(data).unwrap();

                machine_sensors.horizontal_limit_switch = sensor_packet.horizontal_limit_switch;
                machine_sensors.vertical_limit_switch = sensor_packet.vertical_limit_switch;

                match sensor_packet.can_id
                {
                    1=>{
                        horizontal.update(sensor_packet.velocity as i16, sensor_packet.angle as i16, sensor_packet.torque as i16);
                        let motor_sensor = MotorSensor{
                            velocity : horizontal.velocity,
                            position : horizontal.position,
                            torque : horizontal.torque as f32
                        };
                        machine_sensors.horizontal_motor = Some(motor_sensor)
                    },
                    2=>{
                        vertical.update(sensor_packet.velocity as i16, sensor_packet.angle as i16, sensor_packet.torque as i16);
                        let motor_sensor = MotorSensor{
                            velocity : vertical.velocity,
                            position : vertical.position,
                            torque : vertical.torque as f32
                        };
                        machine_sensors.vertical_motor = Some(motor_sensor)
                    },
                    3=>{
                        hand.update(sensor_packet.velocity as i16, sensor_packet.angle as i16, sensor_packet.torque as i16);
                        let motor_sensor = MotorSensor{
                            velocity : hand.velocity,
                            position : hand.position,
                            torque : hand.torque as f32
                        };
                        machine_sensors.hand_motor = Some(motor_sensor)

                    },
                    _=>{

                    }
                }

                machine_sensor_publisher.publish(&machine_sensors).await?;
            }
            Err(_e)=>{

            }
        }
    }
}
