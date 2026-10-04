pub struct RoboMaster
{
    pub velocity : f32,
    pub position : f32,
    pub torque : i16,
    last_count : i16
}

impl RoboMaster {
    pub fn new()->Self
    {
        RoboMaster { velocity: 0.0, position: 0.0, torque: 0, last_count: -1 }
    }

    pub fn update(&mut self, velocity : i16, angle_count : i16, torque : i16)
    {
        self.velocity = velocity as f32;
        self.torque = torque;

        if self.last_count == -1
        {
            self.last_count = angle_count;
            self.position = 0.0;
        }
        else 
        {
            let mut delta_count = angle_count - self.last_count;

            self.last_count = angle_count;

            if delta_count > 4096
            {
                delta_count -= 8192
            }
            else if delta_count < -4096 
            {
                delta_count += 8192
            }

            self.position += delta_count as f32 / 8192.0;
        }

        
    }
}