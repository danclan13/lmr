use std::error::Error;
use std::thread;
use std::time::Duration;
use rpi_embedded::uart::{Parity, Uart};
use rpi_embedded::i2c::I2c;
extern crate pid;
use pid::Pid;
pub const PI: f64 = 3.14159265358979323846264338327950288f64;

fn main() -> Result<(), Box<dyn Error>> {
    let mut uart = Uart::new(115_200, Parity::None, 8, 1)?;
    let mut i2c = I2c::new()?;
    i2c.set_slave_address(0x53)?;

    let mut pidx = Pid::new(2.5, 0.005, 0.02, 97.0, 97.0, 97.0, 97.0, 0.0);
    let mut pidxn = Pid::new(2.5, 0.005, 0.02, 97.0, 97.0, 97.0, 97.0, 0.0);
    //let mut pidy = Pid::new(2.5, 0.005, 0.02, 97.0, 97.0, 97.0, 97.0, 0.0);

    loop {
        
        thread::sleep(Duration::from_millis(10));

        let s = uart.read_line().unwrap_or_default();
        if s.trim().is_empty() == false {
        let spl = s.trim().split(",");
        let vectstr: Vec<&str> = spl.collect();
        if vectstr[0] == "hld" && vectstr[4] == "end"
        {
        let heading = vectstr[1].parse::<f64>().unwrap_or_default();
        let leaning = vectstr[2].parse::<f64>().unwrap_or_default();
        let direction = vectstr[3].parse::<f64>().unwrap_or_default();

        let angle1 = PI/3.0+direction*PI/1800.0;
        let angle2 = PI/3.0-direction*PI/1800.0;
        let angle3 = direction*PI/1800.0;
        
        //let leaning_inv = 180.0 - leaning/10.0;
        //let output = pid.next_control_output(leaning_inv);


        let leaning_xpart = (leaning/10.0)*(angle3.sin());
        //let leaning_ypart = (leaning/10.0)*(angle3.cos());
        let outputx = pidx.next_control_output(leaning_xpart);
        let outputxn = pidxn.next_control_output(-leaning_xpart);
        //let outputy = pidy.next_control_output(leaning_ypart);
        let mut vx = outputx.output;
        if leaning_xpart>0.0 {
            vx = -outputxn.output;
        }
                
        //let vy = outputy.output;
        //let v = (vx.powi(2)+vy.powi(2)).sqrt();

 
        //let v1 = v*(angle1.cos())+80.0;
        //let v2 = v*(angle2.cos())+80.0;
        //let v3 = -1.0*v*(angle3.cos())+80.0;
        
        //let mut buffer_w = [251,v1 as u8,252,v2 as u8,253,v3 as u8,0xA,0xD];  // needs a flush
        //i2c.block_write(0x01, &mut buffer_w).unwrap_or_default();
        println!("Lx: {} Vx: {}", leaning_xpart, vx);}}
    }        
}