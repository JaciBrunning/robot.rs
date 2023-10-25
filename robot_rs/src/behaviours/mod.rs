use std::{cell::RefCell, collections::HashSet, sync::Arc};

use mockall::automock;

pub trait HasBehaviour {
    fn behaviour_key(&self) -> String;
}

pub enum BehaviourState {
    Constructed,
    Running,
    Done,
    Interrupted,
    Errored(anyhow::Error),
}

pub enum BehaviourRequest {
    Continue,
    Finished,
}

pub struct BehaviourProps {
    controlled_systems: HashSet<String>,
}

impl BehaviourProps {
    pub fn controls(&mut self, hb: &dyn HasBehaviour) {
        self.controlled_systems.insert(hb.behaviour_key());
    }
}

#[automock]
pub trait Behaviour {
    fn init(&mut self, ctx: &mut BehaviourProps) -> anyhow::Result<()>;
    fn on_started(&mut self) -> anyhow::Result<BehaviourRequest> {
        Ok(BehaviourRequest::Continue)
    }
    fn on_update(&mut self, dt: f64, total_time: f64) -> anyhow::Result<BehaviourRequest>;
    #[allow(unused_variables)]
    fn on_stopped(&mut self, state: &BehaviourState, total_time: f64) -> anyhow::Result<()> {
        Ok(())
    }
    fn name(&self) -> String {
        "<unnamed behaviour>".into()
    }
}

pub struct ContextualisedBehaviour {
    state: BehaviourState,
    props: BehaviourProps,
    total_time: f64,
    behaviour: Arc<RefCell<dyn Behaviour>>,
}

pub struct BehaviourScheduler {
    running: Vec<ContextualisedBehaviour>,
}

impl BehaviourScheduler {
    pub fn new() -> Self {
        Self { running: vec![] }
    }

    pub fn schedule(&mut self, behaviour: Arc<RefCell<dyn Behaviour>>) -> anyhow::Result<()> {
        let mut contextualised = ContextualisedBehaviour {
            state: BehaviourState::Constructed,
            props: BehaviourProps {
                controlled_systems: HashSet::new(),
            },
            total_time: 0.0,
            behaviour,
        };

        // Initialise the Behaviour
        let name = {
            let mut borrowed = contextualised.behaviour.borrow_mut();
            borrowed.init(&mut contextualised.props)?;
            borrowed.name()
        };

        let wanted = contextualised.props.controlled_systems.clone();

        // Check for any conflicts
        for bhvr in &self.running {
            for sys in &bhvr.props.controlled_systems {
                if wanted.contains(sys) {
                    anyhow::bail!("Behaviour {} wants {}, but a behaviour controlling this system ({}) is already running!", name, sys, bhvr.behaviour.borrow().name());
                }
            }
        }

        // Run the behaviour
        self.running.push(contextualised);

        Ok(())
    }

    pub fn interrupt(&mut self, system_key: &str) -> anyhow::Result<()> {
        for bhvr in &mut self.running {
            if bhvr.props.controlled_systems.contains(system_key) {
                bhvr.state = BehaviourState::Interrupted;
            }
        }

        Ok(())
    }

    pub fn interrupt_all(&mut self) {
        for bhvr in &mut self.running {
            bhvr.state = BehaviourState::Interrupted;
        }
    }

    pub fn update(&mut self, dt: f64) -> anyhow::Result<()> {
        self.running.retain_mut(|bhvr| {
            let mut behaviour = bhvr.behaviour.borrow_mut();
            if let BehaviourState::Constructed = bhvr.state {
                match behaviour.on_started() {
                    Ok(a) => match a {
                        BehaviourRequest::Continue => bhvr.state = BehaviourState::Running,
                        BehaviourRequest::Finished => bhvr.state = BehaviourState::Done,
                    },
                    Err(e) => bhvr.state = BehaviourState::Errored(e),
                }
            }

            if let BehaviourState::Running = bhvr.state {
                match behaviour.on_update(dt, bhvr.total_time) {
                    Ok(a) => match a {
                        BehaviourRequest::Continue => bhvr.total_time += dt,
                        BehaviourRequest::Finished => bhvr.state = BehaviourState::Done,
                    },
                    Err(e) => bhvr.state = BehaviourState::Errored(e),
                }
            }

            match bhvr.state {
                BehaviourState::Done | BehaviourState::Interrupted | BehaviourState::Errored(_) => {
                    match behaviour.on_stopped(&bhvr.state, bhvr.total_time) {
                        Ok(()) => (),
                        Err(e) => bhvr.state = BehaviourState::Errored(e),
                    };
                    false
                }
                _ => true,
            }
        });
        Ok(())
    }
}

#[cfg(test)]
mod test {
    use std::{cell::RefCell, sync::Arc};

    use approx::relative_eq;

    use super::{BehaviourScheduler, BehaviourState, MockBehaviour};

    #[test]
    fn test_scheduler_finish() -> anyhow::Result<()> {
        let mut mock = MockBehaviour::new();
        mock.expect_name().return_once(|| "MockBehaviour".into());

        mock.expect_init()
            .once()
            .withf(move |arg| arg.controlled_systems.is_empty())
            .return_once(move |_| Ok(()));

        mock.expect_on_started()
            .return_once(move || Ok(super::BehaviourRequest::Continue));

        for i in 0..5 {
            mock.expect_on_update()
                .times(1)
                .withf(move |&dt, &total_time| {
                    relative_eq!(dt, 0.05) && relative_eq!(total_time, i as f64 * 0.05)
                })
                .return_once(move |_, _| Ok(super::BehaviourRequest::Continue));
        }
        mock.expect_on_update()
            .once()
            .withf(move |&dt, &total_time| {
                relative_eq!(dt, 0.05) && relative_eq!(total_time, 5.0 * 0.05)
            })
            .return_once(move |_, _| Ok(super::BehaviourRequest::Finished));

        mock.expect_on_stopped()
            .withf(move |state, &total_time| {
                relative_eq!(total_time, 5.0 * 0.05)
                    && match state {
                        BehaviourState::Done => true,
                        _ => false,
                    }
            })
            .return_once(move |_, _| Ok(()));

        let mut scheduler = BehaviourScheduler::new();
        for _ in 0..5 {
            scheduler.update(0.05)?;
        } // Should not have any calls
        scheduler.schedule(Arc::new(RefCell::new(mock)))?;
        for _ in 0..10 {
            scheduler.update(0.05)?;
        } // Should call 5 times

        Ok(())
    }

    #[test]
    fn test_scheduler_interrupted() -> anyhow::Result<()> {
        let mut mock = MockBehaviour::new();
        mock.expect_name().return_once(|| "MockBehaviour".into());

        mock.expect_init()
            .once()
            .withf(move |arg| arg.controlled_systems.is_empty())
            .return_once(move |_| Ok(()));

        mock.expect_on_started()
            .return_once(move || Ok(super::BehaviourRequest::Continue));

        for i in 0..3 {
            mock.expect_on_update()
                .times(1)
                .withf(move |&dt, &total_time| {
                    relative_eq!(dt, 0.05) && relative_eq!(total_time, i as f64 * 0.05)
                })
                .return_once(move |_, _| Ok(super::BehaviourRequest::Continue));
        }

        mock.expect_on_stopped()
            .withf(move |state, &total_time| {
                relative_eq!(total_time, 3.0 * 0.05)
                    && match state {
                        BehaviourState::Interrupted => true,
                        _ => false,
                    }
            })
            .return_once(move |_, _| Ok(()));

        let mut scheduler = BehaviourScheduler::new();
        for _ in 0..5 {
            scheduler.update(0.05)?;
        } // Should not have any calls
        scheduler.schedule(Arc::new(RefCell::new(mock)))?;
        for _ in 0..3 {
            scheduler.update(0.05)?;
        } // Should call 3 times
        scheduler.interrupt_all();
        for _ in 0..3 {
            scheduler.update(0.05)?;
        } // Shouldn't call at all

        Ok(())
    }

    #[test]
    fn test_scheduler_errored() -> anyhow::Result<()> {
        let mut mock = MockBehaviour::new();
        mock.expect_name().return_once(|| "MockBehaviour".into());

        mock.expect_init()
            .once()
            .withf(move |arg| arg.controlled_systems.is_empty())
            .return_once(move |_| Ok(()));

        mock.expect_on_started()
            .return_once(move || Ok(super::BehaviourRequest::Continue));

        for i in 0..5 {
            mock.expect_on_update()
                .times(1)
                .withf(move |&dt, &total_time| {
                    relative_eq!(dt, 0.05) && relative_eq!(total_time, i as f64 * 0.05)
                })
                .return_once(move |_, _| Ok(super::BehaviourRequest::Continue));
        }
        mock.expect_on_update()
            .once()
            .withf(move |&dt, &total_time| {
                relative_eq!(dt, 0.05) && relative_eq!(total_time, 5.0 * 0.05)
            })
            .return_once(move |_, _| Err(anyhow::anyhow!("Some Error")));

        mock.expect_on_stopped()
            .withf(move |state, &total_time| {
                relative_eq!(total_time, 5.0 * 0.05)
                    && match state {
                        BehaviourState::Errored(_) => true,
                        _ => false,
                    }
            })
            .return_once(move |_, _| Ok(()));

        let mut scheduler = BehaviourScheduler::new();
        for _ in 0..5 {
            scheduler.update(0.05)?;
        } // Should not have any calls
        scheduler.schedule(Arc::new(RefCell::new(mock)))?;
        for _ in 0..10 {
            scheduler.update(0.05)?;
        } // Should call 5 times

        Ok(())
    }
}
