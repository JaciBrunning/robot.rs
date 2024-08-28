use std::{
  io::{Read, Write},
  net::ToSocketAddrs,
  num::ParseIntError,
  path::{Path, PathBuf},
};

use anyhow::{anyhow, Result};
use log::info;
use sha1::{Digest, Sha1};
use ssh2::Channel;

pub struct SSHSession {
  session: ssh2::Session,
}

impl SSHSession {
  pub fn connect(addr: impl ToSocketAddrs, user: &str, password: &str) -> Result<SSHSession> {
    let addr = addr
      .to_socket_addrs()?
      .next()
      .ok_or(anyhow!("Invalid Address"))?;
    let user = user.to_owned();
    let password = password.to_owned();

    let tcp = std::net::TcpStream::connect(addr)?;
    let mut session = ssh2::Session::new()?;
    session.set_tcp_stream(tcp);
    session.handshake()?;
    session.userauth_password(&user, &password)?;

    Ok::<SSHSession, anyhow::Error>(SSHSession { session })
  }

  pub fn run(&self, command: &str) -> Result<CommandResult> {
    let mut channel = self.session.channel_session()?;
    self.run_with_channel(&mut channel, command)
  }

  pub fn run_with_channel(&self, channel: &mut Channel, command: &str) -> Result<CommandResult> {
    let command = command.to_owned();
    channel.exec(&command)?;

    let mut s = String::new();
    channel.read_to_string(&mut s)?;
    channel.wait_close()?;

    Ok(CommandResult {
      output: s,
      code: Some(channel.exit_status()?),
    })
  }

  pub fn run_with_stdin(&self, command: &str, stdin: &str) -> Result<CommandResult> {
    let command = command.to_owned();
    let stdin = stdin.to_owned();

    let mut channel = self.session.channel_session()?;
    channel.exec(&command)?;
    channel.write_all(stdin.as_bytes())?;

    let mut s = String::new();
    channel.read_to_string(&mut s)?;
    channel.wait_close()?;

    Ok(CommandResult {
      output: s,
      code: Some(channel.exit_status()?),
    })
  }

  pub fn maybe_copy_file(
    &self,
    file: &PathBuf,
    remote_path: &Path,
    mode: i32,
    checksum: bool,
  ) -> Result<()> {
    let mut channel = self.session.channel_session()?;

    let mut hasher = <Sha1 as Digest>::new();
    hasher.update(std::fs::read(file)?);
    let our_hash = hasher.finalize();

    let their_hash = match self.run_with_channel(
      &mut channel,
      &format!("cat {}.sha1", remote_path.to_str().unwrap()),
    ) {
      Ok(v) if v.success() => decode_hex(&v.output.trim()).ok(),
      _ => None,
    };

    let filename = file.file_name().unwrap().to_str().unwrap();
    match (our_hash, their_hash, checksum) {
      (ours, Some(theirs), true) if ours[..] == theirs[..] => {
        info!("[SSH] {} up to date!", filename);
      }
      _ => {
        info!(
          "[SSH] Deploying: {} -> {}",
          filename,
          remote_path.to_str().unwrap()
        );
        let mut f = std::fs::File::open(file)?;
        let size = f.metadata()?.len();

        let mut send = self.session.scp_send(remote_path, mode, size, None)?;
        std::io::copy(&mut f, &mut send)?;
        send.send_eof()?;
        send.wait_eof()?;

        if checksum {
          self.run(&format!(
            "echo {} > {}.sha1",
            encode_hex(&our_hash[..]),
            remote_path.to_str().unwrap()
          ))?;
        }
      }
    }

    Ok(())
  }
}

#[derive(Clone, Debug)]
pub struct CommandResult {
  output: String,
  code: Option<i32>,
}

impl CommandResult {
  pub fn output(&self) -> String {
    self.output.clone()
  }

  pub fn success(&self) -> bool {
    self.code() == Some(0)
  }

  pub fn code(&self) -> Option<i32> {
    self.code
  }
}

// https://stackoverflow.com/questions/52987181/how-can-i-convert-a-hex-string-to-a-u8-slice
pub fn decode_hex(s: &str) -> Result<Vec<u8>, ParseIntError> {
  (0..s.len())
    .step_by(2)
    .map(|i| u8::from_str_radix(&s[i..i + 2], 16))
    .collect()
}

pub fn encode_hex(bytes: &[u8]) -> String {
  use std::fmt::Write;
  let mut s = String::with_capacity(bytes.len() * 2);
  for &b in bytes {
    write!(&mut s, "{:02x}", b).unwrap();
  }
  s
}
