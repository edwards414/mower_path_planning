//! The XML that a URDF is: elements, attributes, text, comments,
//! processing instructions, a DOCTYPE without an internal subset and CDATA.
//! Enough for what xacro writes and urdfdom reads (tinyxml2's rules where
//! they matter: entities decoded in attribute values, `\r\n` read as `\n`).
//! Namespaces are left in the names, as tinyxml2 leaves them.

use std::fmt;

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Element {
    pub name: String,
    pub attributes: Vec<(String, String)>,
    pub children: Vec<Element>,
}

impl Element {
    pub fn attribute(&self, name: &str) -> Option<&str> {
        self.attributes.iter().find(|(k, _)| k == name).map(|(_, v)| v.as_str())
    }

    /// The child elements called `name`, in document order.
    pub fn children_named<'a>(&'a self, name: &'a str) -> impl Iterator<Item = &'a Element> + 'a {
        self.children.iter().filter(move |c| c.name == name)
    }

    /// tinyxml2's `FirstChildElement(name)`.
    pub fn first_child(&self, name: &str) -> Option<&Element> {
        self.children.iter().find(|c| c.name == name)
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct XmlError {
    pub offset: usize,
    pub message: String,
}

impl fmt::Display for XmlError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "XML error at byte {}: {}", self.offset, self.message)
    }
}

impl std::error::Error for XmlError {}

/// The document's root element.
pub fn parse(text: &str) -> Result<Element, XmlError> {
    let mut p = Parser { s: text.as_bytes(), text, i: 0 };
    if p.s.starts_with("\u{feff}".as_bytes()) {
        p.i = 3;
    }
    p.misc()?;
    if !p.peek_is(b"<") {
        return Err(p.error("no root element"));
    }
    let root = p.element()?;
    p.misc()?;
    if p.i < p.s.len() {
        return Err(p.error("content after the root element"));
    }
    Ok(root)
}

struct Parser<'a> {
    s: &'a [u8],
    text: &'a str,
    i: usize,
}

impl Parser<'_> {
    fn error(&self, message: &str) -> XmlError {
        XmlError { offset: self.i, message: message.to_string() }
    }

    fn peek_is(&self, what: &[u8]) -> bool {
        self.s[self.i..].starts_with(what)
    }

    fn skip_space(&mut self) {
        while self.i < self.s.len() && matches!(self.s[self.i], b' ' | b'\t' | b'\r' | b'\n') {
            self.i += 1;
        }
    }

    /// Past `end`, from here.
    fn skip_past(&mut self, end: &[u8], what: &str) -> Result<(), XmlError> {
        match find(&self.s[self.i..], end) {
            Some(at) => {
                self.i += at + end.len();
                Ok(())
            }
            None => Err(self.error(&format!("unterminated {what}"))),
        }
    }

    /// Whitespace, comments, processing instructions and a DOCTYPE.
    fn misc(&mut self) -> Result<(), XmlError> {
        loop {
            self.skip_space();
            if self.peek_is(b"<!--") {
                self.skip_past(b"-->", "comment")?;
            } else if self.peek_is(b"<?") {
                self.skip_past(b"?>", "processing instruction")?;
            } else if self.peek_is(b"<!DOCTYPE") {
                if find(&self.s[self.i..], b"[").is_some_and(|b| find(&self.s[self.i..], b">").is_some_and(|e| b < e)) {
                    return Err(self.error("a DOCTYPE with an internal subset"));
                }
                self.skip_past(b">", "DOCTYPE")?;
            } else {
                return Ok(());
            }
        }
    }

    fn name(&mut self) -> Result<String, XmlError> {
        let start = self.i;
        while self.i < self.s.len()
            && !matches!(self.s[self.i], b' ' | b'\t' | b'\r' | b'\n' | b'/' | b'>' | b'=' | b'<' | b'"' | b'\'')
        {
            self.i += 1;
        }
        if self.i == start {
            return Err(self.error("a name expected"));
        }
        Ok(self.text[start..self.i].to_string())
    }

    fn element(&mut self) -> Result<Element, XmlError> {
        self.i += 1; // '<'
        let name = self.name()?;
        let mut attributes: Vec<(String, String)> = Vec::new();
        loop {
            self.skip_space();
            if self.peek_is(b"/>") {
                self.i += 2;
                return Ok(Element { name, attributes, children: Vec::new() });
            }
            if self.peek_is(b">") {
                self.i += 1;
                break;
            }
            let key = self.name()?;
            self.skip_space();
            if !self.peek_is(b"=") {
                return Err(self.error(&format!("attribute {key} without a value")));
            }
            self.i += 1;
            self.skip_space();
            let quote = match self.s.get(self.i) {
                Some(q @ (b'"' | b'\'')) => *q,
                _ => return Err(self.error(&format!("attribute {key}: a quoted value expected"))),
            };
            self.i += 1;
            let start = self.i;
            while self.i < self.s.len() && self.s[self.i] != quote {
                if self.s[self.i] == b'<' {
                    return Err(self.error(&format!("attribute {key}: '<' in a value")));
                }
                self.i += 1;
            }
            if self.i >= self.s.len() {
                return Err(self.error(&format!("attribute {key}: unterminated value")));
            }
            let raw = &self.text[start..self.i];
            self.i += 1;
            let value = decode(raw).map_err(|m| XmlError { offset: start, message: m })?;
            if attributes.iter().any(|(k, _)| *k == key) {
                return Err(self.error(&format!("attribute {key} twice")));
            }
            attributes.push((key, value));
        }
        let mut children = Vec::new();
        loop {
            // text up to the next markup, which this model does not keep
            match find(&self.s[self.i..], b"<") {
                Some(at) => self.i += at,
                None => return Err(self.error(&format!("element {name} not closed"))),
            }
            if self.peek_is(b"</") {
                self.i += 2;
                let end = self.name()?;
                if end != name {
                    return Err(self.error(&format!("</{end}> closes <{name}>")));
                }
                self.skip_space();
                if !self.peek_is(b">") {
                    return Err(self.error("'>' expected"));
                }
                self.i += 1;
                return Ok(Element { name, attributes, children });
            } else if self.peek_is(b"<!--") {
                self.skip_past(b"-->", "comment")?;
            } else if self.peek_is(b"<![CDATA[") {
                self.skip_past(b"]]>", "CDATA section")?;
            } else if self.peek_is(b"<?") {
                self.skip_past(b"?>", "processing instruction")?;
            } else {
                children.push(self.element()?);
            }
        }
    }
}

fn find(hay: &[u8], needle: &[u8]) -> Option<usize> {
    hay.windows(needle.len()).position(|w| w == needle)
}

/// Entity references in an attribute value, and `\r\n` read as `\n`.
fn decode(raw: &str) -> Result<String, String> {
    let raw = raw.replace("\r\n", "\n");
    if !raw.contains('&') {
        return Ok(raw);
    }
    let mut out = String::with_capacity(raw.len());
    let mut rest = raw.as_str();
    while let Some(at) = rest.find('&') {
        out.push_str(&rest[..at]);
        let end = rest[at..].find(';').ok_or_else(|| format!("unterminated entity in {raw:?}"))? + at;
        let entity = &rest[at + 1..end];
        let ch = match entity {
            "lt" => '<',
            "gt" => '>',
            "amp" => '&',
            "quot" => '"',
            "apos" => '\'',
            _ if entity.starts_with("#x") || entity.starts_with("#X") => {
                u32::from_str_radix(&entity[2..], 16).ok().and_then(char::from_u32).ok_or_else(|| format!("bad &{entity};"))?
            }
            _ if entity.starts_with('#') => {
                entity[1..].parse::<u32>().ok().and_then(char::from_u32).ok_or_else(|| format!("bad &{entity};"))?
            }
            _ => return Err(format!("unknown entity &{entity};")),
        };
        out.push(ch);
        rest = &rest[end + 1..];
    }
    out.push_str(rest);
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn what_xacro_writes_parses() {
        let doc = r#"<?xml version="1.0" ?><!-- a comment, 中文 --><robot name="r"><!-- x --><link name="a"/>
            <joint name="j" type='fixed'><origin rpy="0 0 1.5708" xyz="0 0 0"/><parent link="a"/><child link="b"/></joint>
            <ros2_control name="R" type="system"><hardware><plugin>p/Q</plugin><param name="d">/dev/x</param></hardware>
            <joint name="j"><state_interface name="position"/></joint></ros2_control><link name="b"/></robot>
            <!-- after -->"#;
        let robot = parse(doc).unwrap();
        assert_eq!(robot.name, "robot");
        assert_eq!(robot.attribute("name"), Some("r"));
        let names: Vec<&str> = robot.children.iter().map(|c| c.name.as_str()).collect();
        assert_eq!(names, ["link", "joint", "ros2_control", "link"]);
        let joint = robot.first_child("joint").unwrap();
        assert_eq!(joint.attribute("type"), Some("fixed"));
        assert_eq!(joint.first_child("origin").unwrap().attribute("rpy"), Some("0 0 1.5708"));
        // the ros2_control block's joint is not a child of robot
        assert_eq!(robot.children_named("joint").count(), 1);
    }

    #[test]
    fn entities_and_cdata() {
        let e = parse("<a v=\"1 &lt; 2 &amp;&#65;&#x42;&quot;\"><![CDATA[<b/>]]><c/></a>").unwrap();
        assert_eq!(e.attribute("v"), Some("1 < 2 &AB\""));
        assert_eq!(e.children.len(), 1);
        assert!(parse("<a v=\"&nope;\"/>").is_err());
    }

    #[test]
    fn broken_documents_are_errors() {
        for bad in ["", "<a>", "<a></b>", "<a v=1/>", "<a/><b/>", "<a v=\"x/>", "<a v=\"1\" v=\"2\"/>", "text"] {
            assert!(parse(bad).is_err(), "{bad:?}");
        }
        assert!(parse("<!DOCTYPE robot><robot/>").is_ok());
    }
}
