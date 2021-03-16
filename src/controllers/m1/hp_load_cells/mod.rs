use crate::{
    build_controller, build_inputs, build_outputs, import_simulink,
    io::{jar, Tags},
    IOTags, DOS, IO,
};

import_simulink!(M1HPloadcells, U : (M1_HP_D,84,M1_HP_cmd,42), Y : (M1_HP_LC,42));
build_inputs!(M1HpD, 84, M1HpCmd, 42);
build_outputs!(M1HpLc, 42);
build_controller!(M1HPloadcells,
                  U : (M1_HP_D -> (M1HpD,m1_hp_d),
                       M1_HP_cmd -> (M1HpCmd,m1_hp_cmd) ),
                  Y : (M1_HP_LC -> (M1HpLc,m1_hp_lc))
);

impl<'a> IOTags for Controller<'a> {
    fn outputs_tags(&self) -> Vec<Tags> {
        vec![jar::M1HPLC::new()]
    }
    fn inputs_tags(&self) -> Vec<Tags> {
        vec![jar::OSSHardpointD::new(), jar::M1HPCmd::new()]
    }
}
impl<'a> DOS for Controller<'a> {
    fn inputs(&mut self, data: Vec<IO<Vec<f64>>>) -> Result<&mut Self, Box<dyn std::error::Error>> {
        if data.into_iter().fold(2, |mut a, io| {
            match io {
                IO::OSSHardpointD { data: Some(values) } => {
                    for (k, v) in values.into_iter().enumerate() {
                        self.m1_hp_d[k] = v;
                    }
                    a -= 1;
                }
                IO::M1HPCmd { data: Some(values) } => {
                    for (k, v) in values.into_iter().enumerate() {
                        self.m1_hp_cmd[k] = v;
                    }
                    a -= 1;
                }
                _ => (),
            }
            if a == 0 {
                return a;
            }
            a
        }) == 0
        {
            Ok(self)
        } else {
            Err("Either mount drive controller OSSHardpointD or M1HPCmd not found".into())
        }
    }
    fn outputs(&mut self) -> Option<Vec<IO<Vec<f64>>>> {
        Some(vec![IO::M1HPLC {
            data: Some(Vec::<f64>::from(&self.m1_hp_lc)),
        }])
    }
}
