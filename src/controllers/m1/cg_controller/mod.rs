use crate::{
    build_controller, build_inputs, build_outputs, import_simulink,
    io::{jar, Tags},
    IOTags, DOS, IO,
};

import_simulink!(M1OFL_Control, U : (HP_LC,42), Y : (M1_Rel_F,42));
build_inputs!(M1HPLC, 42);
build_outputs!(M1CGFM, 42);
build_controller!(M1OFL_Control,
                  U : (HP_LC -> (M1HPLC,m1_hp_lc)),
                  Y : (M1_Rel_F -> (M1CGFM,m1_cg_fm))
);

impl<'a> IOTags for Controller<'a> {
    fn outputs_tags(&self) -> Vec<Tags> {
        vec![jar::M1CGFM::new()]
    }
    fn inputs_tags(&self) -> Vec<Tags> {
        vec![jar::M1HPLC::new()]
    }
}
impl<'a> DOS for Controller<'a> {
    fn inputs(&mut self, data: Vec<IO<Vec<f64>>>) -> Result<&mut Self, Box<dyn std::error::Error>> {
        if data.into_iter().fold(1, |mut a, io| {
            match io {
                IO::M1HPLC { data: Some(values) } => {
                    for (k, v) in values.into_iter().enumerate() {
                        self.m1_hp_lc[k] = v;
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
        Some(vec![IO::M1CGFM {
            data: Some(Vec::<f64>::from(&self.m1_cg_fm)),
        }])
    }
}
