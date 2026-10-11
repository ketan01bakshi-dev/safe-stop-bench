"""A private certificate authority and a broker certificate for TLS (v2.30).

    .venv\\Scripts\\python.exe -m fleet.certs make --host 192.168.1.6       make secrets/tls/{ca,server}.{pem,key}
    .venv\\Scripts\\python.exe -m fleet.certs show                          what is in them, and the CA's fingerprint

How the boards use it: the CA certificate (public) is uploaded to the board over USB (scripts/provision_wifi.py) and PINNED there. The board
then trusts exactly one thing: a broker certificate signed by THIS CA, for the address it was told to connect to. Nobody who does not hold
ca.key can make the board talk to an impostor, and no public certificate authority is involved.

  ca.pem      the CA certificate: public, goes to the board and to the PC clients
  ca.key      the CA's private key: SECRET, never leaves this PC; only needed to issue another broker certificate
  server.pem  the broker's certificate, signed by the CA      server.key  its private key: SECRET, goes only to the broker

Two design choices that look odd and are deliberate:

* **Valid from 1970-01-01.** The ESP32 has no real-time clock. After a power-up it believes it is 1 January 1970 until it learns the time,
  and TLS refuses a certificate that is "not yet valid". A certificate that starts in 1970 is accepted by a board that does not know the
  date. The price: the board cannot notice an expired certificate. The end date is 2049 (the last year X.509 can write in its short form);
  renew by running `make` again and re-uploading the CA.
* **The broker's address appears twice.** Python and OpenSSL match an IP address against the certificate's IP entries; the board's TLS
  library (mbedTLS) matches the text it was given against name entries. Each address is therefore listed as an IP entry AND as a name entry.
"""
from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import ipaddress
import sys
from pathlib import Path

from cryptography import x509
from cryptography.hazmat.primitives import hashes, serialization
from cryptography.hazmat.primitives.asymmetric import rsa
from cryptography.x509.oid import ExtendedKeyUsageOID, NameOID

from ssb.config import ROOT

DEFAULT_DIR = ROOT / "secrets" / "tls"
NOT_BEFORE = dt.datetime(1970, 1, 1)           # see the docstring
NOT_AFTER = dt.datetime(2049, 12, 31, 23, 59, 59)


def _key() -> rsa.RSAPrivateKey:
    return rsa.generate_private_key(public_exponent=65537, key_size=2048)


def _name(cn: str) -> x509.Name:
    return x509.Name([x509.NameAttribute(NameOID.ORGANIZATION_NAME, "safe-stop bench"), x509.NameAttribute(NameOID.COMMON_NAME, cn)])


def make_ca(common_name: str = "safe-stop bench private CA") -> tuple[x509.Certificate, rsa.RSAPrivateKey]:
    key = _key()
    name = _name(common_name)
    cert = (x509.CertificateBuilder().subject_name(name).issuer_name(name).public_key(key.public_key())
            .serial_number(x509.random_serial_number()).not_valid_before(NOT_BEFORE).not_valid_after(NOT_AFTER)
            .add_extension(x509.BasicConstraints(ca=True, path_length=0), critical=True)
            .add_extension(x509.KeyUsage(digital_signature=True, key_cert_sign=True, crl_sign=True, content_commitment=False, key_encipherment=False,
                                         data_encipherment=False, key_agreement=False, encipher_only=False, decipher_only=False), critical=True)
            .add_extension(x509.SubjectKeyIdentifier.from_public_key(key.public_key()), critical=False)
            .sign(key, hashes.SHA256()))
    return cert, key


def san_entries(hosts: list[str]) -> list[x509.GeneralName]:
    """Every host as the kind of entry its form needs, and every IP address ALSO as a name entry (see the docstring)."""
    out: list[x509.GeneralName] = []
    seen: set[str] = set()
    for h in [*hosts, "localhost", "127.0.0.1"]:
        if h in seen:
            continue
        seen.add(h)
        try:
            out.append(x509.IPAddress(ipaddress.ip_address(h)))
        except ValueError:
            pass
        out.append(x509.DNSName(h))
    return out


def make_server(ca_cert: x509.Certificate, ca_key: rsa.RSAPrivateKey, hosts: list[str]) -> tuple[x509.Certificate, rsa.RSAPrivateKey]:
    if not hosts:
        raise ValueError("at least one host (the address the boards connect to) is required")
    key = _key()
    cert = (x509.CertificateBuilder().subject_name(_name(hosts[0])).issuer_name(ca_cert.subject).public_key(key.public_key())
            .serial_number(x509.random_serial_number()).not_valid_before(NOT_BEFORE).not_valid_after(NOT_AFTER)
            .add_extension(x509.BasicConstraints(ca=False, path_length=None), critical=True)
            .add_extension(x509.KeyUsage(digital_signature=True, key_encipherment=True, key_cert_sign=False, crl_sign=False, content_commitment=False,
                                         data_encipherment=False, key_agreement=False, encipher_only=False, decipher_only=False), critical=True)
            .add_extension(x509.ExtendedKeyUsage([ExtendedKeyUsageOID.SERVER_AUTH]), critical=False)
            .add_extension(x509.SubjectAlternativeName(san_entries(hosts)), critical=False)
            .add_extension(x509.AuthorityKeyIdentifier.from_issuer_public_key(ca_key.public_key()), critical=False)
            .sign(ca_key, hashes.SHA256()))
    return cert, key


def pem(cert: x509.Certificate) -> bytes:
    return cert.public_bytes(serialization.Encoding.PEM)


def key_pem(key: rsa.RSAPrivateKey) -> bytes:
    return key.private_bytes(serialization.Encoding.PEM, serialization.PrivateFormat.TraditionalOpenSSL, serialization.NoEncryption())


def fingerprint(cert_pem: bytes) -> str:
    """SHA-256 of the DER form, colon-separated: what a person compares to check that the board and the PC hold the same CA."""
    der = x509.load_pem_x509_certificate(cert_pem).public_bytes(serialization.Encoding.DER)
    h = hashlib.sha256(der).hexdigest().upper()
    return ":".join(h[i:i + 2] for i in range(0, len(h), 2))


def write_all(out: Path, hosts: list[str], force: bool = False) -> dict[str, Path]:
    paths = {n: out / n for n in ("ca.pem", "ca.key", "server.pem", "server.key")}
    if any(p.exists() for p in paths.values()) and not force:
        raise FileExistsError(f"{out} already holds certificates: use --force to replace them (every board must then be given the new ca.pem)")
    out.mkdir(parents=True, exist_ok=True)
    ca, ca_key = make_ca()
    srv, srv_key = make_server(ca, ca_key, hosts)
    paths["ca.pem"].write_bytes(pem(ca))
    paths["ca.key"].write_bytes(key_pem(ca_key))
    paths["server.pem"].write_bytes(pem(srv))
    paths["server.key"].write_bytes(key_pem(srv_key))
    return paths


def describe(path: Path) -> str:
    c = x509.load_pem_x509_certificate(path.read_bytes())
    san = c.extensions.get_extension_for_class(x509.SubjectAlternativeName).value if any(e.oid == x509.SubjectAlternativeName.oid for e in c.extensions) else None
    names = ", ".join(str(g.value) for g in san) if san else "-"
    return (f"{path.name}: {c.subject.rfc4514_string()}\n  valid {c.not_valid_before_utc:%Y-%m-%d} to {c.not_valid_after_utc:%Y-%m-%d}; "
            f"names: {names}\n  sha256 {fingerprint(path.read_bytes())}")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    mk = sub.add_parser("make", help="create a CA and a broker certificate")
    mk.add_argument("--host", action="append", required=True, help="the address boards connect to (this PC's LAN address); repeat for more")
    mk.add_argument("--out", type=Path, default=DEFAULT_DIR)
    mk.add_argument("--force", action="store_true")
    sh = sub.add_parser("show", help="describe the certificates in a folder")
    sh.add_argument("--out", type=Path, default=DEFAULT_DIR)
    a = ap.parse_args()
    if a.cmd == "make":
        try:
            paths = write_all(a.out, a.host, a.force)
        except FileExistsError as e:
            print(e)
            return 1
        print(f"written to {a.out}:")
        for n in paths:
            print(f"  {n:11} {'SECRET: keep on this PC' if n.endswith('.key') else 'public' if n == 'ca.pem' else 'goes to the broker'}")
        print(describe(paths["ca.pem"]))
        print(describe(paths["server.pem"]))
        return 0
    for n in ("ca.pem", "server.pem"):
        if not (a.out / n).exists():
            print(f"{a.out / n} is missing: run `python -m fleet.certs make --host <this PC's address>` first")
            return 1
        print(describe(a.out / n))
    return 0


if __name__ == "__main__":
    sys.exit(main())
