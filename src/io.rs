//! DOS inputs/outputs
//!
//! Provides the definitions for all the inputs and outputs used by DOS

use super::{wind_loads,DOSError};
use core::fmt::Debug;
use serde::Serialize;

#[derive(Clone, Debug)]
pub enum IOError {
    Missing(String),
    Empty,
    FileNotFound,
    PickleRead,
    Outputs,
}

macro_rules! build_io {
    ($($variant:ident),+) => {
        /// Inputs/Outputs definition
        #[derive(Debug,Clone,Serialize)]
        pub enum IO<T> {
            $($variant{data: Option<T>}),+
        }
        impl IO<usize> {
            /// Assign `n` to `IO` `data`
            pub fn assign(&mut self, n: usize) {
                match self {
                    $(IO::$variant{ data: values} => {*values=Some(n);}),+
                }
            }
        }
        impl<T> PartialEq<IO<T>> for IO<()> {
            fn eq(&self, other: &IO<T>) -> bool {
                match (self,other) {
                    $((IO::$variant{..},IO::$variant{..}) => true,)+
                    _ => false,
                }
            }
        }
        impl<T,U> From<&IO<U>> for IO<T> {
            /// Converts a `IO<T>` into an `Option<T>`
            fn from(io: &IO<U>) -> Self {
                match io {
                    $(IO::$variant{ ..} => IO::$variant{ data: Default::default()}),+
                }
            }
        }
        impl<T,U: Iterator<Item=T>> From<&mut IO<U>> for Option<IO<T>> {
            /// Converts a `IO<T>` into an `Option<T>`
            fn from(io: &mut IO<U>) -> Self {
                match io {
                    $(IO::$variant{ data: Some(data)} => match data.next() {
                        Some(data) => Some(IO::$variant{ data: Some(data)}),
                        None => None
                    },)+
                        $(IO::$variant{ data: None} => None,)+
                }
            }
        }
        impl<T> From<IO<T>> for Option<T> {
            /// Converts a `IO<T>` into an `Option<T>`
            fn from(io: IO<T>) -> Self {
                match io {
                    $(IO::$variant{ data: values} => values),+
                }
            }
        }
        impl<T> From<(&IO<()>,Option<T>)> for IO<T> {
            fn from((io,data): (&IO<()>,Option<T>)) -> Self {
                match io {
                    $(IO::$variant{ .. } => IO::$variant{ data: data}),+
                }
            }
        }
        impl<T: Debug> From<IO<T>> for Result<T,Box<dyn std::error::Error>> {
            /// Converts a `IO<T>` into an `Option<T>`
            fn from(io: IO<T>) -> Self {
                match io {
                    $(IO::$variant{ data: values} => values.ok_or_else(|| format!("{:?} data missing",IO::<T>::$variant{data: None}).into())),+
                }
            }
        }
        impl<T: Clone> From<&IO<T>> for Option<T> {
            /// Converts a `&IO<T>` into an `Option<T>`
            fn from(io: &IO<T>) -> Self {
                match io {
                    $(IO::$variant{ data: values} => values.as_ref().cloned()),+
                }
            }
        }
        impl From<(&IO<usize>,Vec<f64>)> for IO<Vec<f64>> {
            /// Converts a `(&IO<usize>,Vec<f64>)` into an `IO<Vec<f64>>`
            fn from((io,v): (&IO<usize>,Vec<f64>)) -> Self {
                match io {
                    $(IO::$variant{ data: _} => IO::$variant{ data: Some(v)}),+
                }
            }
        }
        impl From<(&IO<()>,Vec<f64>)> for IO<Vec<f64>> {
            /// Converts a `(&IO<()>,Vec<f64>)` into an `IO<Vec<f64>>`
            fn from((io,v): (&IO<()>,Vec<f64>)) -> Self {
                match io {
                    $(IO::$variant{ data: _} => IO::$variant{ data: Some(v)}),+
                }
            }
        }
        pub mod jar {
            //! A DOS Inputs/Outputs builder
            use super::IO;
            $(pub struct $variant {}
              impl $variant {
                  /// Creates a new `IO` type variant with `data` set to `None`
                  pub fn new<T>() -> IO<T> {
                      IO::$variant{ data: None}
                  }
                  /// Creates a new `IO` type variant filled with `data`
                  pub fn with<T>(data: T) -> IO<T> {
                      IO::$variant{ data: Some(data)}
                  }
              }
            )+
        }
    };
}
macro_rules! io_match_fem {
    (inputs: ($($inputs_variant:ident),+), outputs: ($($outputs_variant:ident),+)) => {
        impl<T: Debug> IO<T> {
            /// Matches a FEM input to a DOS `IO` returning the FEM input value
            pub fn match_fem_inputs(&self, fem_inputs: &fem::fem_io::Inputs) -> Option<Vec<fem::IO>> {
                match (self,fem_inputs) {
                    $((IO::$inputs_variant{data: _}, fem::fem_io::Inputs::$inputs_variant(v)) => {
                        Some(v.clone())},)+
                    (_, _) => None,
                }
            }
            /// Matches a FEM output to a DOS `IO` returning the FEM output value
            pub fn match_fem_outputs(&self, fem_outputs: &fem::fem_io::Outputs) -> Option<Vec<fem::IO>> {
                match (self,fem_outputs) {
                    $((IO::$outputs_variant{data: _}, fem::fem_io::Outputs::$outputs_variant(v)) => Some(v.clone()),)+
                        (_, _) => None,
                }
            }
        }
    };
}
macro_rules! io_match_wind_loads {
    ($($variant:ident),+) => {
        impl<T> IO<T> {
            /// Matches a wind loads to a DOS `IO` returning the wind load value as an iterator over the first `n` elements
            pub fn data(&self, wind_loads: &wind_loads::Loads) -> Option<std::vec::IntoIter<Vec<f64>>> {
                match (self,wind_loads) {
                    $((IO::$variant{..}, wind_loads::Loads::$variant(v)) => Some(v.clone().into_iter()),)+
                        (_, _) => None,
                }
            }
            /// Matches a wind loads to a DOS `IO` returning the wind load value as an iterator
            pub fn ndata(&self, wind_loads: &wind_loads::Loads, n: usize) -> Option<std::vec::IntoIter<Vec<f64>>> {
                match (self,wind_loads) {
                    $((IO::$variant{..}, wind_loads::Loads::$variant(v)) => Some(v[..n].to_owned().into_iter()),)+
                        (_, _) => None,
                }
            }
        }
    };
}

/// A type for empty `IO`
pub type Tags = IO<()>;

build_io!(
    // Uniform Wind Pressure
    OSSTopEnd6F,  // Top-End
    MCM2Lcl6F,    // M2 segments
    OSSTruss6F,   //Truss
    OSSM1Lcl6F,   // M1 segments
    OSSCellLcl6F, // M1 segment cells
    OSSGIR6F,     // GIR
    OSSCRING6F,   // C-rings
    // Axial wind forces on M1 mirror segments
    M1DistributedWindf,
    // Axial displacement of M1 segment surface nodes
    M1Segment1AxialD,
    M1Segment2AxialD,
    M1Segment3AxialD,
    M1Segment4AxialD,
    M1Segment5AxialD,
    M1Segment6AxialD,
    M1Segment7AxialD,
    // M1 hardpoints
    OSSHarpointDeltaF, // forces
    OSSHardpointD,     // displacements
    // M1 Actuators forces applied to back-side of M1 segments
    M1ActuatorsSegment1,
    M1ActuatorsSegment2,
    M1ActuatorsSegment3,
    M1ActuatorsSegment4,
    M1actuatorsSegment5,
    M1actuatorsSegment6,
    M1ActuatorsSegment7,
    // Mount Drives
    OSSAzDriveF,  // azimuth drive
    OSSElDriveF,  // elevation drive
    OSSGIRDriveF, // GIR drive
    OSSAzDriveD,
    OSSElDriveD,
    OSSGIRDriveD,
    // Mount Drives
    OSSAzDriveTorque,
    OSSElDriveTorque,
    OSSRotDriveTorque,
    OSSAzEncoderAngle,
    OSSElEncoderAngle,
    OSSRotEncoderAngle,
    // Azimuth, elevation, rotation drive torques
    SlewTorques,
    // Line of sight
    OSSM1LOS,  // M1
    MCM2LOS6D, // M2
    // Base of Pier
    OSSBASE6F,
    // Inertial Measurement Units
    OSSIMUs6d,
    // M2 Positioners
    MCM2SmHexF,
    // ASM Proof Mass Actuators
    MCM2PMA1F,
    // ASM
    MCM2CP6F,  // Cold plates
    MCM2RB6F,  // Reference bodies
    MCM2CP6D,  // Cold plates
    MCM2RB6D,  // Reference bodies
    MCM2Lcl6D, // Face sheets
    //
    MCM2TE6F,
    MCM2TEIF6F,
    OSSTrussTEIF6f,
    MCM2GravCS0,
    MCM2PZTS1F,
    MCM2PZTS2F,
    MCM2PZTS3F,
    MCM2PZTS4F,
    MCM2PZTS5F,
    MCM2PZTS6F,
    MCM2PZTS7F,
    MCM2SmallS16F,
    MCM2SmallS26F,
    MCM2SmallS36F,
    MCM2SmallS46F,
    MCM2SmallS56F,
    MCM2SmallS66F,
    MCM2SmallS76F,
    OSSGravCS0,
    OSSTrussIF6D,
    OSSGIR6D,
    OSSCRING6D,
    OSSBASE6D,
    OSSM1Lcl,
    OSSTruss6d,
    OSSCellLcl,
    MCM2SmallS16D,
    MCM2PZTS1D,
    MCM2SmallS26D,
    MCM2PZTS2D,
    MCM2SmallS36D,
    MCM2PZTS3D,
    MCM2SmallS46D,
    MCM2PZTS4D,
    MCM2SmallS56D,
    MCM2PZTS5D,
    MCM2SmallS66D,
    MCM2PZTS6D,
    MCM2SmallS76D,
    MCM2PZTS7D,
    M1SurfacesD,
    M1EdgeSensors,
    MCM2CP1D,
    MCM2SmHexD,
    M2edgesensors,
    MCM2TEIF6D,
    MCM2TE6D,
    M2ReferenceBody1AxialD,
    M2ReferenceBody2AxialD,
    M2ReferenceBody3AxialD,
    M2ReferenceBody4AxialD,
    M2ReferenceBody5AxialD,
    M2ReferenceBody6AxialD,
    M2ReferenceBody7AxialD,
    MountCmd,
    // M1 control
    M1HPCmd,
    M1HPLC,
    M1CGFM
);

io_match_fem!(
    inputs:
        (
            MCM2RB6F,
            MCM2TE6F,
            OSSTopEnd6F,
            OSSTruss6F,
            OSSGIR6F,
            OSSCRING6F,
            OSSCellLcl6F,
            OSSM1Lcl6F,
            MCM2Lcl6F,
            OSSAzDriveF,
            OSSElDriveF,
            OSSGIRDriveF,
            OSSHarpointDeltaF,
            OSSAzDriveTorque,
            OSSElDriveTorque,
            OSSRotDriveTorque
        ),
    outputs:
        (
            OSSAzDriveD,
            OSSElDriveD,
            OSSGIRDriveD,
            OSSM1Lcl,
            MCM2Lcl6D,
            OSSHardpointD,
            OSSAzEncoderAngle,
            OSSElEncoderAngle,
            OSSRotEncoderAngle,
            MCM2RB6D
        )
);
io_match_wind_loads!(
    OSSTopEnd6F,
    OSSTruss6F,
    OSSGIR6F,
    OSSCRING6F,
    OSSCellLcl6F,
    OSSM1Lcl6F,
    MCM2Lcl6F
);
