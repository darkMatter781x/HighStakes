use std::{num::NonZeroU32, time::Duration};

use anyhow::Context;
use tokio::{
    io::{stdout, AsyncReadExt, AsyncWriteExt},
    process::Command,
    time::sleep,
};
use vex_v5_qemu_host::{
    brain::{Brain, Program},
    devices::{distance_sensor::DistanceSensor, imu::Imu, rotation_sensor::RotationSensor},
    protocol::{distance_sensor::DistanceObject, imu::ImuObject, rotation_sensor::RotationObject},
};

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let kernel = "/workspaces/HighStakes/sim/vex-v5-qemu/target/armv7a-none-eabi/debug/kernel";
    let bin = "../bin/monolith.bin";
    let mut brain = Brain::new();
    let peripherals = brain.peripherals.take().unwrap();

    let mut dist = DistanceSensor::new(peripherals.port_11);
    let mut vert_rot = RotationSensor::new(peripherals.port_20);
    let mut hori_rot = RotationSensor::new(peripherals.port_19);
    let mut imu = Imu::new(peripherals.port_7);

    tokio::task::spawn(async move {
        let mut pos: i32 = 0;
        loop {
            pos += 100;

            dist.set_status(1).await;
            dist.set_object(Some(DistanceObject {
                distance: pos.unsigned_abs(),
                relative_size: pos.unsigned_abs(),
                velocity: pos.into(),
                confidence: pos.unsigned_abs(),
            })).await;
            vert_rot.set_status(1).await;
            hori_rot.set_status(1).await;
            imu.set_status(1).await;
            // dist.set_status(1).await;
            vert_rot
                .set_object(Some(RotationObject { position: pos }))
                .await;
            hori_rot
                .set_object(Some(RotationObject { position: pos }))
                .await;
            imu.set_object(Some(ImuObject {
                temperature: 1000.0,
                heading: (pos / 10).into(),
                degrees: 10.,
                mode: 1,
            }))
            .await;

            sleep(Duration::from_secs(1)).await;
        }
    });

    let mut qemu = Command::new("qemu-system-arm");
    qemu.args(["-S", "-s"]);
    brain
        .run_program(
            qemu,
            kernel.into(),
            Program {
                path: bin.into(),
                load_addr: NonZeroU32::new(0x03800000).unwrap(),
            },
            None,
        )
        .await
        .context("Failed to start QEMU.")?;

    let usb_task = tokio::task::spawn(async move {
        let mut usb = peripherals.usb;
        let mut out = stdout();

        loop {
            let mut buf = vec![0; 1024];
            let n = usb.read(&mut buf).await.unwrap();
            if n == 0 {
                break;
            }

            out.write_all(&buf[..n]).await.unwrap();
            out.flush().await.unwrap();
        }
    });

    brain.wait_for_exit().await?;
    usb_task.abort();

    Ok(())
}
