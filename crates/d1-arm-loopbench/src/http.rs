//! The three REST calls the bench makes (take, renew and release the arm
//! lease), over a plain HTTP/1.1 connection to the daemon's loopback port.

use std::{
    io::{Read, Write},
    net::TcpStream,
    time::Duration,
};

use serde_json::Value;

/// One request; the response status and its body parsed as JSON.
///
/// # Errors
///
/// A URL that is not `http://host:port`, a connection or I/O failure, or a
/// body that is not JSON.
pub fn request(base: &str, method: &str, path: &str, body: &Value) -> Result<(u16, Value), String> {
    let authority = base
        .strip_prefix("http://")
        .ok_or_else(|| format!("{base} is not an http:// URL"))?
        .trim_end_matches('/');
    let mut stream =
        TcpStream::connect(authority).map_err(|error| format!("connect {authority}: {error}"))?;
    stream
        .set_read_timeout(Some(Duration::from_secs(10)))
        .map_err(|error| error.to_string())?;
    let body = body.to_string();
    let head = format!(
        "{method} {path} HTTP/1.1\r\nHost: {authority}\r\nContent-Type: application/json\r\n\
         Content-Length: {}\r\nConnection: close\r\n\r\n",
        body.len()
    );
    stream
        .write_all(head.as_bytes())
        .and_then(|()| stream.write_all(body.as_bytes()))
        .map_err(|error| format!("send {method} {path}: {error}"))?;
    let mut response = Vec::new();
    stream
        .read_to_end(&mut response)
        .map_err(|error| format!("read {method} {path}: {error}"))?;
    let text = String::from_utf8_lossy(&response);
    let (head, payload) = text
        .split_once("\r\n\r\n")
        .ok_or_else(|| format!("{method} {path}: no HTTP response"))?;
    let status = head
        .split_whitespace()
        .nth(1)
        .and_then(|code| code.parse().ok())
        .ok_or_else(|| format!("{method} {path}: no status in {head:?}"))?;
    let chunked = head.lines().any(|line| {
        line.to_ascii_lowercase()
            .starts_with("transfer-encoding: chunked")
    });
    let payload = if chunked {
        dechunk(payload)
    } else {
        payload.to_owned()
    };
    let value = serde_json::from_str(&payload)
        .map_err(|error| format!("{method} {path}: body is not JSON ({error}): {payload}"))?;
    Ok((status, value))
}

fn dechunk(mut payload: &str) -> String {
    let mut out = String::new();
    while let Some((size, rest)) = payload.split_once("\r\n") {
        let Ok(size) = usize::from_str_radix(size.trim(), 16) else {
            break;
        };
        if size == 0 || rest.len() < size {
            break;
        }
        out.push_str(&rest[..size]);
        payload = rest[size..].trim_start_matches("\r\n");
    }
    out
}

#[cfg(test)]
mod tests {
    #[test]
    fn a_chunked_body_is_joined() {
        assert_eq!(
            super::dechunk("4\r\n{\"a\"\r\n3\r\n:1}\r\n0\r\n\r\n"),
            "{\"a\":1}"
        );
    }
}
