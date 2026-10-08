pub struct PidConfig {
    p_gain: f32,
    i_gain: f32,
    d_gain: f32,
    max_output: f32,
    min_output: f32,
    max_integral: f32,
}

impl PidConfig {
    pub fn new(
        p_gain: f32,
        i_gain: f32,
        d_gain: f32,
        max_output: f32,
        min_output: f32,
        max_integral: f32,
    ) -> Self {
        Self {
            p_gain,
            i_gain,
            d_gain,
            max_output,
            min_output,
            max_integral,
        }
    }
}

pub struct PID {
    config: PidConfig,
    integral: f32,
    enable_integral_reset: bool,
    previous_mesurement: f32,
    previous_target: f32,
}

impl PID {
    pub fn new(config: PidConfig) -> Self {
        Self {
            config,
            integral: 0.0,
            enable_integral_reset: false,
            previous_mesurement: 0.0,
            previous_target: 0.0,
        }
    }

    /// PIDコントローラーの積分項をリセットするかどうかを設定します。
    /// enableがtrueの場合、積分項は目標値の符号が反転したときにリセットされます。
    /// enableがfalseの場合、積分項はリセットされません。
    pub fn enable_integral_reset(&mut self, enable: bool) {
        self.enable_integral_reset = enable;
    }

    pub fn compute(&mut self, target: f32, now: f32, delta_time: f32) -> f32 {
        // 目標値と現在の値の差を計算します。
        let error = target - now;

        // 積分項のリセットが有効な場合、目標値の符号が反転したときに積分項をリセットします。
        if self.enable_integral_reset {
            if target > 0.0 && self.previous_target < 0.0
                || target < 0.0 && self.previous_target > 0.0
            {
                self.integral = 0.0;
            }
        }
        // 前回の目標値を更新します。
        self.previous_target = target;

        // 積分項を更新します。積分項はエラーと時間の積で計算されます。
        self.integral += error * delta_time;
        // 積分項が最大値を超えないようにクランプ(最大値を超えるなら最大値にする)します。
        self.integral = Self::clamp(
            self.integral,
            self.config.max_integral,
            -self.config.max_integral,
        );

        // 微分項を計算します。微分項は時間あたりの偏差の変化率で計算されます。
        let derivative = (error - self.previous_mesurement) / delta_time;
        // 前回の測定値を更新します。
        self.previous_mesurement = error;

        // PIDコントローラーの出力を計算します。出力は比例項、積分項、微分項の和で計算されます。
        let output = self.config.p_gain * error
            + self.config.i_gain * self.integral
            + self.config.d_gain * derivative;

        // 出力が最大値を超えないようにクランプ(最大値を超えるなら最大値にする)します。
        Self::clamp(output, self.config.max_output, self.config.min_output)
    }

    pub fn compute_d_on_mesuarement(&mut self, target: f32, now: f32, delta_time: f32) -> f32 {
        // 目標値と現在の値の差を計算します。
        let error = target - now;

        // 積分項のリセットが有効な場合、目標値の符号が反転したときに積分項をリセットします。
        if self.enable_integral_reset {
            if target > 0.0 && self.previous_target < 0.0
                || target < 0.0 && self.previous_target > 0.0
            {
                self.integral = 0.0;
            }
        }
        // 前回の目標値を更新します。
        self.previous_target = target;

        // 積分項を更新します。積分項はエラーと時間の積で計算されます。
        self.integral += error * delta_time;
        // 積分項が最大値を超えないようにクランプ(最大値を超えるなら最大値にする)します。
        self.integral = Self::clamp(
            self.integral,
            self.config.max_integral,
            -self.config.max_integral,
        );

        // 微分項を計算します。微分項は時間あたりの測定値の変化率で計算されます。
        let derivative = (now - self.previous_mesurement) / delta_time;
        // 前回の測定値を更新します。
        self.previous_mesurement = now;

        // PIDコントローラーの出力を計算します。出力は比例項、積分項、微分項の和で計算されます。
        let output = self.config.p_gain * error
            + self.config.i_gain * self.integral
            + self.config.d_gain * derivative;

        // 出力が最大値を超えないようにクランプ(最大値を超えるなら最大値にする)します。
        Self::clamp(output, self.config.max_output, self.config.min_output)
    }

    fn clamp(value: f32, max: f32, min: f32) -> f32 {
        if value > max {
            max
        } else if value < min {
            min
        } else {
            value
        }
    }
}