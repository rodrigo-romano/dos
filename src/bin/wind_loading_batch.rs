use dos::{
    controllers::{m1, mount::pdr as mount, state_space::DiscreteStateSpace},
    io::jar::*,
    io::IO,
    DataLogging, WindLoads, DOS,
};
use fem::FEM;
use rayon::prelude::*;
use serde_pickle as pkl;
use simple_logger::SimpleLogger;
use std::{error::Error, fs::File, path::Path, time::Instant};

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

fn job(cfd_case: &str) -> Result<(), Box<dyn Error>> {
    SimpleLogger::new().init().unwrap();
    /*let job_idx = env::var("AWS_BATCH_JOB_ARRAY_INDEX")
    .expect("AWS_BATCH_JOB_ARRAY_INDEX env var missing")
    .parse::<usize>()
    .expect("AWS_BATCH_JOB_ARRAY_INDEX parsing failed");*/
    let fem_data_path = Path::new("/fsx").join("Baseline2020");
    // WIND LOADS
    let tic = Timer::tic();
    println!(
        "Loading wind loads from {:?}...",
        fem_data_path.join(cfd_case)
    );
    //let n_sample = 20 * 1000;
    let mut wind_loading =
        WindLoads::from_pickle(fem_data_path.join(cfd_case).join("wind_loads_2kHz.pkl"))?
            .range(0.0, 400.0)
            .decimate(2)
            .truss()?
            .m2_asm_topend()?
            .m1_segments()?
            .m1_cell()?
            .m2_asm_reference_bodies()?
            .build()?;
    tic.print_toc();
    // MOUNT CONTROL
    let mut mnt_drives = mount::drives::Controller::new();
    let mut mnt_ctrl = mount::controller::Controller::new();

    // M1
    let mut m1_hardpoints = m1::hp_load_cells::Controller::new();
    let mut m1_ctrl = m1::cg_controller::Controller::new();

    // FEM
    let sampling_rate = 1e3;
    let m1_rbm = OSSM1Lcl::new();
    let m2_rbm = MCM2RB6D::new();
    let tic = Timer::tic();
    println!("Building FEM dynamic model...");
    let mut fem = DiscreteStateSpace::from(FEM::from_pickle(
        fem_data_path.join("20210225_1447_MT_mount_v202102_ASM_wind2.pkl"),
    )?)
    //.dump_eigen_frequencies(fem_data_path.join("eigen_frequencies.pkl"))
    .sampling(sampling_rate)
    .proportional_damping(2. / 100.)
    .max_eigen_frequency(75.0)
    .inputs_from(&wind_loading)
    .inputs_from(&mnt_drives)
    .outputs(vec![m1_rbm.clone(), m2_rbm.clone()])
    .outputs(vec![
        OSSAzEncoderAngle::new(),
        OSSElEncoderAngle::new(),
        OSSRotEncoderAngle::new(),
    ])
    .outputs(vec![OSSHardpointD::new()])
    .build()?;
    tic.print_toc();

    // DATA LOGGING
    let mut data = DataLogging::new()
        .sampling_rate(sampling_rate)
        //.key(m1_rbm.clone())
        //.key(m2_rbm.clone())
        .build();

    println!("Running model ...");
    let tic = Timer::tic();
    let mut mount_drives_forces = Some(vec![
        OSSAzDriveTorque::with(vec![0f64; 12]),
        OSSElDriveTorque::with(vec![0f64; 4]),
        OSSRotDriveTorque::with(vec![0f64; 4]),
    ]);
    let mut m1_cg_fm: Option<Vec<IO<Vec<f64>>>> = None;
    // FEEDBACK LOOP
    let mut k = 0;
    while let Some(mut fem_forces) = wind_loading.outputs() {
        // FEM
        mount_drives_forces.as_mut().map(|x| {
            fem_forces.append(x);
        });
        m1_cg_fm.as_ref().map(|x| {
            fem_forces[OSSM1Lcl6F::new()] += &x[0];
            fem_forces[OSSCellLcl6F::new()] -= &x[0];
        });
        let fem_outputs = fem.in_step_out(fem_forces)?.ok_or("FEM output is empty")?;
        // MOUNT CONTROLLER & DRIVES
        let mount_encoders = &fem_outputs[2..5];
        mount_drives_forces = mnt_ctrl
            .in_step_out(mount_encoders.to_vec())?
            .and_then(|mut x| {
                x.extend_from_slice(mount_encoders);
                Some(mnt_drives.in_step_out(x.to_owned()))
            })
            .unwrap()?;
        // M1 HARDPOINT & CG CONTROLLER
        if k % 10 == 0 {
            let mut m1_hp = vec![M1HPCmd::with(vec![0f64; 42])];
            m1_hp.extend_from_slice(&[fem_outputs[OSSHardpointD::new()].clone()]);
            m1_cg_fm = m1_hardpoints
                .in_step_out(m1_hp)?
                .and_then(|x| Some(m1_ctrl.in_step_out(x)))
                .unwrap()?;
        }
        // DATA LOGGING
        data.step()?;
        data.log(&fem_outputs[0])?.log(&fem_outputs[1])?;
        k += 1;
    }
    tic.print_toc();

    // OUTPUTS SAVING
    let mut f = File::create(
        fem_data_path.join("windloading.20210225_1447_MT_mount_v202102_ASM_wind2.pkl"),
    )
    .unwrap();
    pkl::to_writer(
        &mut f,
        &[
            data.time_series(m1_rbm),
            data.time_series(m2_rbm),
            data.time_series(M1HPLC::new()),
            data.time_series(M1CGFM::new()),
            //data.time_series(OSSAzEncoderAngle::new()),
            //data.time_series(OSSElEncoderAngle::new()),
            //data.time_series(OSSRotEncoderAngle::new()),
            //data.time_series(MountCmd::new()),
        ],
        true,
    )
    .unwrap();
    Ok(())
}

fn main() {
    let cfd_cases = vec![
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
    cfd_cases
        .into_par_iter()
        .for_each(|cfd_case| match job(cfd_case) {
            Ok(_) => println!("{} succeed!!!", cfd_case),
            Err(_) => println!("{} failed!?!", cfd_case),
        })
}
