use std::fmt;

/// The main types of DOS errors
#[derive(Debug)]
pub enum DOSError<T: fmt::Debug> {
    Component(T),
    Outputs,
    Inputs,
    Step,
    IO(Box<dyn std::error::Error>),
}

impl<T: fmt::Debug> From<std::io::Error> for DOSError<T> {
    fn from(e: std::io::Error) -> DOSError<T> {
        DOSError::IO(Box::new(e))
    }
}

impl<T: fmt::Debug> From<serde_pickle::Error> for DOSError<T> {
    fn from(e: serde_pickle::Error) -> DOSError<T> {
        DOSError::IO(Box::new(e))
    }
}

impl<T: fmt::Debug> fmt::Display for DOSError<T> {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        use DOSError::*;
        match self {
            Inputs => write!(f, "DOS Inputs failed"),
            Outputs => write!(f, "DOS Outputs failed"),
            Step => write!(f, "DOS Step failed"),
            Component(component) => component.fmt(f),
            IO(error) => error.fmt(f),
        }
    }
}

impl<T: fmt::Debug> std::error::Error for DOSError<T> {}
