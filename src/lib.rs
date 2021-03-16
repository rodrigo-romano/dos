//! Dynamic optical simulation library for the Giant Magellan Telescope
//!
//! The library is used to build end-to-end simulations of the GMT.
//! It provides the following features:
//!  - the state space representation of the finite element model of the telescope,
//!  - the optical model of the telescope including ray tracing through the telescope,
//!  - the controllers of the different telescope subsystem,
//!  - the wind loads that are applied to the telescope.
//!
//! An end-to-end simulation is divided into components, each represented by a structure:
//!  - [`DiscreteModalsolver`](crate::controllers::state_space::DiscreteStateSpace) for the finite element model of the telescope ([example](crate::controllers::state_space)),
//!  - [`WindLoading`] for the wind loads,
//!  - `Controller` for each subsystem controller.
//!
//! Each component structure contains a [`Vec`] of either inputs, outputs or both that corresponds to some variant of the [`IO`] enum type.
//! Each component structure must implement the [`Iterator`] and the [`DOS`] traits.
//! The [`next`](core::iter::Iterator::next) method of the [`Iterator`] trait is used to update the state of the component at each time step.
//! The [`inputs`](crate::DOS::inputs) method of the [`DOS`] trait passes inputs data to the components whereas the [`outputs`](crate::DOS::outputs) method returns the component outputs.

pub mod controllers;
pub mod io;
pub mod telltale;
pub mod wind_loads;
pub mod error;

use error::DOSError;
use fem;
use io::IO;
#[doc(inline)]
pub use telltale::DataLogging;
#[doc(inline)]
pub use wind_loads::{WindLoading, WindLoads};

/// Used to get the list of inputs or outputs
pub trait IOTags {
    /// Return the list of outputs
    fn outputs_tags(&self) -> Vec<IO<()>>;
    /// Return the list of inputs
    fn inputs_tags(&self) -> Vec<IO<()>>;
}
/// Used to glue together the different components of an end-to-end model
pub trait DOS {
    /// Computes and returns a vector outputs from a model component
    fn outputs(&mut self) -> Option<Vec<IO<Vec<f64>>>>;
    /// Passes a vector of input data to a model component
    fn inputs(&mut self, data: Vec<IO<Vec<f64>>>) -> Result<&mut Self, Box<dyn std::error::Error>>;
    /// Updates the state of a model component for one time step
    fn step(&mut self) -> Result<&mut Self, DOSError<()>>
    where
        Self: Sized + Iterator,
    {
        self.next().and(Some(self)).ok_or_else(|| DOSError::Step)
    }
    /// Combines `inputs`, `step` and `outputs` in a single method
    ///
    /// This is equivalent to `.inputs(...)?.step()?.outputs()?`
    fn in_step_out(
        &mut self,
        data: Vec<IO<Vec<f64>>>,
    ) -> Result<Option<Vec<IO<Vec<f64>>>>, Box<dyn std::error::Error>>
    where
        Self: Sized + Iterator,
    {
        Ok(self.inputs(data)?.step()?.outputs())
    }
}
