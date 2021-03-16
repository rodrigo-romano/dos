//! This module is used to create the structure that applied wind forces and moments on the telescope FEM
//!
//! There are 7 wind load sources applied to the following elements of the telescope structure:
//!  - the C-Rings
//!  - the GIR
//!  - the M1 cells
//!  - the M1 segments
//!  - the trusses
//!  - the M2 segments
//!  - the top-end

use super::{
    io::{jar, Tags},
    DOSError, IOTags, DOS, IO,
};
use serde;
use serde::Deserialize;
use serde_pickle as pkl;
use std::fs::File;
use std::io::BufReader;
use std::path::Path;

#[derive(Clone, Debug)]
pub enum WindLoadsError {
    Len,
    Empty,
    FileNotFound,
    PickleRead,
    Outputs,
}

type Result<T> = std::result::Result<T, DOSError<WindLoadsError>>;
type Outputs = Option<std::vec::IntoIter<Vec<f64>>>;

macro_rules! loads {
    ($($name:expr, $variant:ident),+) => {
        /// Wind loads forces and moments
        ///
        /// A time vector containing vectors of forces and moments
        #[derive(Deserialize, Debug,Clone)]
        pub enum Loads {
            $(#[serde(rename = $name)]
              $variant(Vec<Vec<f64>>)),+
        }
        impl Loads {
            /// Returns the number of samples in the time series
            pub fn len(&self) -> usize {
                match self {
                    $(Loads::$variant(io) => io.len()),+
                }
            }
            /// Return the loads
            pub fn io(self) -> Vec<Vec<f64>> {
                match self {
                    $(Loads::$variant(io) => io),+
                }
            }
        }
    };
}
loads!(
    "OSS_TopEnd_6F",
    OSSTopEnd6F,
    "OSS_Truss_6F",
    OSSTruss6F,
    "OSS_GIR_6F",
    OSSGIR6F,
    "OSS_CRING_6F",
    OSSCRING6F,
    "OSS_Cell_lcl_6F",
    OSSCellLcl6F,
    "OSS_M1_lcl_6F",
    OSSM1Lcl6F,
    "MC_M2_lcl_force_6F",
    MCM2Lcl6F
);

/// Wind loads builder
///
/// This structure is used to read the forces and moments time series from a data file and to create the [`WindLoading`] structure
#[derive(Deserialize)]
pub struct WindLoads {
    /// forces and moments time series
    #[serde(rename = "outputs")]
    pub loads: Vec<Option<Loads>>,
    /// time vector
    pub time: Vec<f64>,
    #[serde(skip)]
    n_sample: Option<usize>,
    #[serde(skip)]
    tagged_loads: Vec<IO<std::vec::IntoIter<Vec<f64>>>>,
}

impl WindLoads {
    /// Reads the wind loads from a pickle file
    pub fn from_pickle<P: AsRef<Path>>(path: P) -> Result<Self> {
        let f = File::open(path)?;
        let r = BufReader::with_capacity(1_000_000_000, f);
        let v: serde_pickle::Value = serde_pickle::from_reader(r)?;
        Ok(pkl::from_value(v)?)
    }
    /// Returns the number of samples in the time series
    fn len(&self) -> Result<usize> {
        self.loads
            .iter()
            .find_map(|x| x.as_ref().and_then(|x| Some(x.len())))
            .ok_or(DOSError::Component(WindLoadsError::Len))
    }
    fn tagged_load(&self, io: &Tags) -> Result<Outputs> {
        match &self.n_sample {
            Some(n) => self
                .loads
                .iter()
                .find_map(|x| x.as_ref().and_then(|x| io.ndata(x, *n)))
                .map_or(Err(DOSError::Component(WindLoadsError::Empty)), |x| {
                    Ok(Some(x))
                }),
            None => self
                .loads
                .iter()
                .find_map(|x| x.as_ref().and_then(|x| io.data(x)))
                .map_or(Err(DOSError::Component(WindLoadsError::Empty)), |x| {
                    Ok(Some(x))
                }),
        }
    }
    /// Set the number of time sample
    pub fn n_sample(self, n_sample: usize) -> Result<Self> {
        assert!(n_sample > 0, "n_sample must be greater than 0");
        let n = self.len()?;
        assert!(
            n_sample <= n,
            format!(
                "n_sample cannot be greater than the number of sample ({})",
                n
            )
        );
        Ok(Self {
            n_sample: Some(if n_sample <= n { n_sample } else { n }),
            ..self
        })
    }
    /// Selects loads on the truss
    pub fn truss(mut self) -> Result<Self> {
        self.tagged_loads.push(IO::OSSTruss6F {
            data: self.tagged_load(&jar::OSSTruss6F::new())?,
        });
        Ok(self)
    }
    /// Selects loads on the top-end
    pub fn topend(mut self) -> Result<Self> {
        self.tagged_loads.push(IO::OSSTopEnd6F {
            data: self.tagged_load(&jar::OSSTopEnd6F::new())?,
        });
        Ok(self)
    }
    pub fn m2_asm_topend(mut self) -> Result<Self> {
        self.tagged_loads.push(IO::MCM2TE6F {
            data: self.tagged_load(&jar::OSSTopEnd6F::new())?,
        });
        Ok(self)
    }
    /// Selects loads on the C-ring
    pub fn cring(mut self) -> Result<Self> {
        self.tagged_loads.push(IO::OSSCRING6F {
            data: self.tagged_load(&jar::OSSCRING6F::new())?,
        });
        Ok(self)
    }
    /// Selects loads on the GIR
    pub fn gir(mut self) -> Result<Self> {
        self.tagged_loads.push(IO::OSSGIR6F {
            data: self.tagged_load(&jar::OSSGIR6F::new())?,
        });
        Ok(self)
    }
    /// Selects loads on the M1 cells
    pub fn m1_cell(mut self) -> Result<Self> {
        self.tagged_loads.push(IO::OSSCellLcl6F {
            data: self.tagged_load(&jar::OSSCellLcl6F::new())?,
        });
        Ok(self)
    }
    /// Selects loads on the M1 segments
    pub fn m1_segments(mut self) -> Result<Self> {
        self.tagged_loads.push(IO::OSSM1Lcl6F {
            data: self.tagged_load(&jar::OSSM1Lcl6F::new())?,
        });
        Ok(self)
    }
    /// Selects loads on the M2 segments
    pub fn m2_segments(mut self) -> Result<Self> {
        self.tagged_loads.push(IO::MCM2Lcl6F {
            data: self.tagged_load(&jar::MCM2Lcl6F::new())?,
        });
        Ok(self)
    }
    pub fn m2_asm_reference_bodies(mut self) -> Result<Self> {
        self.tagged_loads.push(IO::MCM2RB6F {
            data: self.tagged_load(&jar::MCM2Lcl6F::new())?,
        });
        Ok(self)
    }
    /// Selects all loads
    pub fn select_all(self) -> Result<Self> {
        self.topend()?
            .m2_segments()?
            .truss()?
            .m1_segments()?
            .m1_cell()?
            .gir()?
            .cring()
    }
    /// Selects all loads in the ASM configuration
    pub fn select_all_with_asm(self) -> Result<Self> {
        self.m2_asm_topend()?
            .m2_asm_reference_bodies()?
            .truss()?
            .m1_segments()?
            .m1_cell()?
            .gir()?
            .cring()
    }
    /// Builds a wind loading source object
    pub fn build(self) -> Result<WindLoading> {
        Ok(WindLoading {
            n_sample: self.n_sample.unwrap_or(self.len()?),
            loads: self.tagged_loads,
        })
    }
}

/// Wind loading sources
///
/// This structure contains the time series of wind forces and moments.
/// The time series implement the [`Iterator`] trait and the [`outputs`](crate::wind_loads::WindLoading::outputs) method step through the iterator
#[derive(Default)]
pub struct WindLoading {
    pub loads: Vec<IO<std::vec::IntoIter<Vec<f64>>>>,
    pub n_sample: usize,
}

/// Wind loading interface
impl IOTags for WindLoading {
    fn outputs_tags(&self) -> Vec<Tags> {
        self.loads.iter().map(|x| x.into()).collect()
    }
    fn inputs_tags(&self) -> Vec<Tags> {
        unimplemented!("WindLoading takes no inputs")
    }
}
impl DOS for WindLoading {
    fn inputs(
        &mut self,
        _: Vec<IO<Vec<f64>>>,
    ) -> std::result::Result<&mut Self, Box<dyn std::error::Error>> {
        unimplemented!()
    }
    fn outputs(&mut self) -> Option<Vec<IO<Vec<f64>>>> {
        self.loads
            .iter_mut()
            .map(|x| -> Option<IO<Vec<f64>>> { x.into() })
            .collect()
    }
}
