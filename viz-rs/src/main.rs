use std::{
    io::{BufRead, BufReader},
    process::{Child, ChildStdout, Command, Stdio},
};

use anyhow::anyhow;

use itertools::Itertools as _;

use rerun::{
    external::{arrow2::{self, io::print}, re_types},
    ComponentBatch, RecordingStream, RecordingStreamError,
};

struct Message<'a> {
    topic: &'a str,
    data: &'a str,
}

fn parse_line<'a>(line: &'a str) -> Option<Message<'a>> {
    // line should be formatted like
    // [<topic>]:<data>
    let mut parts = line.split(['[', ']']);
    let _ = parts.next();
    let topic = parts.next();
    let data = parts.next().and_then(|raw| raw.strip_prefix(':'));
    topic.and_then(|t| data.and_then(|d| Some(Message { topic: t, data: d })))
}

struct MessageSubscriber {
    //# Topic that this subscriber is subscribed to
    topic: String,
    //# Listener
    listener: Box<dyn FnMut(&str) -> ()>,
}

impl MessageSubscriber {
    fn new<S: Into<String>>(topic: S, callback: impl FnMut(&str) -> () + 'static) -> Self {
        MessageSubscriber {
            topic: topic.into(),
            listener: Box::new(callback),
        }
    }
}

struct MessageHandler {
    subs: Vec<MessageSubscriber>,
}
impl MessageHandler {
    fn new() -> Self {
        MessageHandler { subs: Vec::new() }
    }
    fn handle(&mut self, msg: Message) {
        let mut handled_at_least_once = false;
        for s in self.subs.as_mut_slice() {
            if s.topic == msg.topic {
                handled_at_least_once = true;
                (s.listener)(msg.data);
            };
        }
        if !handled_at_least_once {
            println!("unhandled: [{}]{}", msg.topic, msg.data);
        }
    }
    fn sub<S: Into<String>>(
        &mut self,
        topic: S,
        callback: impl FnMut(&str) -> () + 'static,
    ) -> &mut Self {
        self.subs.push(MessageSubscriber::new(topic, callback));
        self
    }
}

/// Runs simulated sensor program on v5 brain
fn run_sim_on_brain() -> Result<Child, std::io::Error> {
    Command::new("sh")
        .arg("-c")
        .arg("v5 run --file ../bin/sim/monolith.bin -s 7 -i question-mark")
        .stdout(Stdio::piped())
        .stderr(Stdio::inherit())
        .spawn()
}
/// Runs real sensor program on v5 brain
fn run_real_on_brain() -> Result<Child, std::io::Error> {
    Command::new("sh")
        .arg("-c")
        .arg("pros ut --slot 6 --after run")
        .stdout(Stdio::piped())
        .stderr(Stdio::inherit())
        .spawn()
}

/// Runs program in v5 sim
fn run_simulator() -> Result<Child, std::io::Error> {
    Command::new("sh")
        .arg("-c")
        .arg("simulator monolith ../bin/sim/monolith.bin")
        .stdout(Stdio::piped())
        .stderr(Stdio::inherit())
        .spawn()
}

fn handle_stdout(
    stdout: ChildStdout,
    handler: &mut MessageHandler,
) -> Result<(), Box<dyn std::error::Error>> {
    let reader = BufReader::new(stdout);

    for line_res in reader.lines() {
        match line_res {
            Ok(line) => {
                match parse_line(line.as_str()) {
                    Some(msg) => handler.handle(msg),
                    None => println!("[Program] {}", line),
                };
            }
            Err(err) => return Err(Box::new(err)),
        };
    }
    Ok(())
}

struct Particle {
    x: f32,
    y: f32,
    weight: f32,
}

/// A custom [component bundle] that extends Rerun's builtin [`rerun::Points2D`] archetype with extra
/// [`rerun::Component`]s.
///
/// [component bundle]: [`AsComponents`]
struct RerunParticles {
    points2d: rerun::Points2D,
    confidences: Vec<Confidence>,
}

impl RerunParticles {
    pub fn new<T: IntoIterator<Item = Particle>>(particles: T) -> Self {
        let (coords, weights): (Vec<_>, Vec<_>) = particles
            .into_iter()
            .map(|p| ((p.x, p.y), p.weight))
            .unzip();
        let max = weights
            .clone()
            .into_iter()
            .max_by(|a, b| a.partial_cmp(b).expect("ordering failed!?"))
            .expect("max weight failed!?");
        let (colors, confidences): (Vec<rerun::Color>, Vec<Confidence>) = weights
            .into_iter()
            .map(|w| -> (rerun::Color, Confidence) {
                (
                    rerun::Color::from_unmultiplied_rgba(
                        0x00,
                        0x00,
                        0xB0,
                        ((w / max) as f32 * (0xFF as f32)) as u8,
                    ),
                    w.into(),
                )
            })
            .unzip();
        RerunParticles {
            points2d: rerun::Points2D::new(coords).with_colors(colors),
            confidences,
        }
    }
}

impl rerun::AsComponents for RerunParticles {
    fn as_component_batches(&self) -> Vec<rerun::ComponentBatchCowWithDescriptor<'_>> {
        let indicator = rerun::NamedIndicatorComponent("user.ParticlesIndicator".into());
        self.points2d
            .as_component_batches()
            .into_iter()
            .chain(
                [
                    indicator.to_batch(),
                    rerun::ComponentBatchCowWithDescriptor::new(
                        &self.confidences as &dyn rerun::ComponentBatch,
                    )
                    // Optionally override the descriptor with extra information.
                    .with_descriptor_override(
                        self.confidences
                            .descriptor()
                            .into_owned()
                            .or_with_archetype_name(|| "user.Particles".into())
                            .or_with_archetype_field_name(|| "confidences".into()),
                    ),
                ]
                .into_iter(),
            )
            .collect()
    }
}

/// A custom [`rerun::Component`] that is backed by a builtin [`rerun::Float32`] scalar.
#[derive(Debug, Clone, Copy)]
struct Confidence(rerun::Float32);

impl From<f32> for Confidence {
    fn from(v: f32) -> Self {
        Self(rerun::Float32(v))
    }
}

impl rerun::SizeBytes for Confidence {
    #[inline]
    fn heap_size_bytes(&self) -> u64 {
        0
    }
}

impl rerun::Loggable for Confidence {
    #[inline]
    fn arrow2_datatype() -> arrow2::datatypes::DataType {
        rerun::Float32::arrow2_datatype()
    }

    #[inline]
    fn to_arrow2_opt<'a>(
        data: impl IntoIterator<Item = Option<impl Into<std::borrow::Cow<'a, Self>>>>,
    ) -> re_types::SerializationResult<Box<dyn arrow2::array::Array>>
    where
        Self: 'a,
    {
        rerun::Float32::to_arrow2_opt(data.into_iter().map(|opt| opt.map(Into::into).map(|c| c.0)))
    }
}

impl rerun::Component for Confidence {
    #[inline]
    fn descriptor() -> rerun::ComponentDescriptor {
        rerun::ComponentDescriptor::new("user.Confidence")
    }
}

fn parse_particles(rec: &RecordingStream, raw_str: &str) {
    // pStr is formatted like:
    // "<micros>;<x_float_1>,<y_float_1>,<weight_float_1>;<x_float_2>,<y_float_2>,<weight_float_2>;..."
    let body = || -> anyhow::Result<()> {
        let (micros_str, rest) = raw_str
            .split_once(';')
            .ok_or(anyhow!("Could not split with ';': {}", raw_str))?;
        let micros: i64 = micros_str.parse()?;
        // println!("parse_particles: string after removing micros {rest}");

        rec.set_time_nanos("capture", micros * 1000i64);
        rec.log(
            "particles",
            &RerunParticles::new(
                rest.split(';')
                    .filter_map(|particle_str| -> Option<Particle> {
                        let particle_opt = particle_str
                            .split(',')
                            .map(|s| s.parse::<f32>().ok())
                            .collect_tuple::<(Option<f32>, Option<f32>, Option<f32>)>(
                        )?;
                        particle_opt
                            .0
                            .zip(particle_opt.1)
                            .zip(particle_opt.2)
                            .map(|((x, y), weight)| Particle { x, y, weight })
                    }),
            ),
        )?;
        // println!("sent data!");
        Ok(())
    };
    if let Err(err) = body() {
        println!("Could not parse particle string. Error: {err}")
    }
}

fn add_background(rec: &RecordingStream) -> Result<(), RecordingStreamError> {
    let image = include_bytes!("../h2h.png");

    rec.log_static(
        "background/image",
        &rerun::EncodedImage::from_file_contents(image.to_vec()),
    )?;

    Ok(())
}

fn add_field_grid(rec: &RecordingStream) -> Result<(), RecordingStreamError> {
    let border = [
        (-72., -72.),
        (72., -72.),
        (72., 72.),
        (-72., 72.),
        (-72., -72.),
    ];
    rec.log_static("field/border", &rerun::LineStrips2D::new([border]))?;

    Ok(())
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("{}", rerun::default_server_addr());
    let rec = rerun::RecordingStreamBuilder::new("781xMCL").connect_tcp()?;

    add_field_grid(&rec)?;
    // add_background(&rec)?;

    let handler = &mut MessageHandler::new();

    handler.sub("TELEMETRY.PF.PARTICLES", move |s| parse_particles(&rec, s));

    // let child_process = run_real_on_brain()?;
    let child_process = run_sim_on_brain()?;
    // let child_process = run_simulator()?;
    let child_stdout = child_process
        .stdout
        .expect("child process's stdout was not available!?");
    handle_stdout(child_stdout, handler)?;

    Ok(())
}
