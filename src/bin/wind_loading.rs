use dos::{
    controllers::{mount, state_space::DiscreteStateSpace},
    io::jar::*,
    DataLogging, WindLoads, DOS,
};
use fem::FEM;
use serde_pickle as pkl;
use std::{env, error::Error, fs::File, time::Instant};

struct Timer {
    time: Instant,
}
impl Timer {
    pub fn tic() -> Self {
        Self {
            time: Instant::now(),
        }
    }
    pub fn toc(self) -> f64 {
        self.time.elapsed().as_secs_f64()
    }
    pub fn print_toc(self) {
        println!("... in {:3}s", self.toc());
    }
}

fn main() -> Result<(), Box<dyn Error>> {
    let cfd_case = vec![
        "b2019_0z_0az_os_2ms",
        "b2019_0z_0az_os_7ms",
        "b2019_0z_0az_cd_12ms",
        "b2019_0z_0az_cd_17ms",
        "b2019_0z_45az_os_2ms",
        "b2019_0z_45az_os_7ms",
        "b2019_0z_45az_cd_12ms",
        "b2019_0z_45az_cd_17ms",
        "b2019_0z_90az_os_2ms",
        "b2019_0z_90az_os_7ms",
        "b2019_0z_90az_cd_12ms",
        "b2019_0z_90az_cd_17ms",
        "b2019_0z_135az_os_2ms",
        "b2019_0z_135az_os_7ms",
        "b2019_0z_135az_cd_12ms",
        "b2019_0z_135az_cd_17ms",
        "b2019_0z_180az_os_2ms",
        "b2019_0z_180az_os_7ms",
        "b2019_0z_180az_cd_12ms",
        "b2019_0z_180az_cd_17ms",
        "b2019_30z_0az_os_2ms",
        "b2019_30z_0az_os_7ms",
        "b2019_30z_0az_cd_12ms",
        "b2019_30z_0az_cd_17ms",
        "b2019_30z_45az_os_2ms",
        "b2019_30z_45az_os_7ms",
        "b2019_30z_45az_cd_12ms",
        "b2019_30z_45az_cd_17ms",
        "b2019_30z_90az_os_2ms",
        "b2019_30z_90az_os_7ms",
        "b2019_30z_90az_cd_12ms",
        "b2019_30z_90az_cd_17ms",
        "b2019_30z_135az_os_2ms",
        "b2019_30z_135az_os_7ms",
        "b2019_30z_135az_cd_12ms",
        "b2019_30z_135az_cd_17ms",
        "b2019_30z_180az_os_2ms",
        "b2019_30z_180az_os_7ms",
        "b2019_30z_180az_cd_12ms",
        "b2019_30z_180az_cd_17ms",
        "b2019_60z_0az_os_2ms",
        "b2019_60z_0az_os_7ms",
        "b2019_60z_0az_cd_12ms",
        "b2019_60z_0az_cd_17ms",
        "b2019_60z_45az_os_2ms",
        "b2019_60z_45az_os_7ms",
        "b2019_60z_45az_cd_12ms",
        "b2019_60z_45az_cd_17ms",
        "b2019_60z_90az_os_2ms",
        "b2019_60z_90az_os_7ms",
        "b2019_60z_90az_cd_12ms",
        "b2019_60z_90az_cd_17ms",
        "b2019_60z_135az_os_2ms",
        "b2019_60z_135az_os_7ms",
        "b2019_60z_135az_cd_12ms",
        "b2019_60z_135az_cd_17ms",
        "b2019_60z_180az_os_2ms",
        "b2019_60z_180az_os_7ms",
        "b2019_60z_180az_cd_12ms",
        "b2019_60z_180az_cd_17ms",
    ];
    let job_idx = env::var("AWS_BATCH_JOB_ARRAY_INDEX")
        .expect("AWS_BATCH_JOB_ARRAY_INDEX env var missing")
        .parse::<usize>()
        .expect("AWS_BATCH_JOB_ARRAY_INDEX parsing failed");
    // WIND LOADS
    let datapath = format!("/fsx/Baseline2020/{}",cfd_case[job_idx]);
    let tic = Timer::tic();
    println!("Loading wind loads {}...",cfd_case[job_idx]);
    let n_sample = 2000 * 400;
    let mut wind_loading =
        WindLoads::from_pickle(&format!("{}/wind_loads_2kHz.pkl",datapath))?
            .n_sample(n_sample)?
            .select_all()?
            .build()?;
    tic.print_toc();

    // MOUNT CONTROL
    let mut mnt_drives = mount::drives::Controller::new();
    let mut mnt_ctrl = mount::controller::Controller::new();

    // FEM
    let sampling_rate = 2e3;
    let m1_rbm = OSSM1Lcl::new();
    let m2_rbm = MCM2Lcl6D::new();
    let tic = Timer::tic();
    println!("Building FEM dynamic model...");
    let mut fem = DiscreteStateSpace::from(FEM::from_pickle(
        "/fsx/Baseline2020/mt_fsm/modal_state_space_model_2ndOrder.pkl",
    )?)
    .sampling(sampling_rate)
    .inputs_from(&wind_loading)
    .inputs_from(&mnt_drives)
    .outputs(vec![m1_rbm.clone(), m2_rbm.clone()])
    .outputs_to(&mnt_ctrl)
    .build()?;
    tic.print_toc();

    // DATA LOGGING
    let mut data = DataLogging::new()
        .sampling_rate(2e3)
        //.key(m1_rbm.clone())
        //.key(m2_rbm.clone())
        .build();

    println!("Sample #: {}", wind_loading.n_sample);
    println!("Running model ...");
    let tic = Timer::tic();

    // FEEDBACK LOOP
    let mut mount_drives_cmd = None;
    while let Some(mut fem_forces) = wind_loading.outputs() {
        data.step()?;
        // Mount Drives
        mnt_drives
            .inputs(mount_drives_cmd.unwrap_or(vec![
                MountCmd::with(vec![0f64; 3]),
                OSSAzDriveD::with(vec![0f64; 8]),
                OSSElDriveD::with(vec![0f64; 8]),
                OSSGIRDriveD::with(vec![0f64; 4]),
            ]))?
            .step()?
            .outputs()
            .map(|mut x| {
                fem_forces.append(&mut x);
            });
        // FEM
        let ys = fem
            .inputs(fem_forces)?
            .step()?
            .outputs()
            .ok_or("FEM output is empty")?;
        // Mount Controller
        mount_drives_cmd = mnt_ctrl
            .inputs(ys[2..].to_vec())?
            .step()?
            .outputs()
            .and_then(|mut x| {
                x.extend_from_slice(&ys[2..]);
                Some(x)
            });
        data.log(&ys[0])?.log(&ys[1])?;
    }
    tic.print_toc();

    // OUTPUTS SAVING
    let mut f = File::create(&format!("{}/wind_loading.data.pkl",datapath)).unwrap();
    pkl::to_writer(
        &mut f,
        &[data.time_series(m1_rbm), data.time_series(m2_rbm)],
        true,
    )
    .unwrap();
    Ok(())
}
