use dos::{controllers::state_space::DiscreteStateSpace, io::jar, DOS};
use fem::FEM;
use std::path::Path;
use simple_logger::SimpleLogger;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    SimpleLogger::new().init().unwrap();
    let sampling_rate = 1e3; // Hz
    let fem_data_path = Path::new("data").join("20210225_1447_MT_mount_v202102_ASM_wind2");
    let fem = FEM::from_pickle(fem_data_path.join("modal_state_space_model_2ndOrder.pkl"))?;
    let mut fem_ss = DiscreteStateSpace::from(fem)
        .sampling(sampling_rate)
        .proportional_damping(2. / 100.)
        .max_eigen_frequency(75.0) // Hz
        .inputs(vec![jar::OSSM1Lcl6F::new()])
        .outputs(vec![jar::OSSM1Lcl::new()])
        .build()?;
    let y = fem_ss
        .inputs(vec![jar::OSSM1Lcl6F::with(vec![0f64; 42])])?
        .step()?
        .outputs();
    assert_eq!(
        Option::<Vec<f64>>::from(&y.unwrap()[0]).unwrap()
            .iter()
            .sum::<f64>(),
        0f64
    );
    Ok(())
}
