#!/bin/sh
# Install a prebuilt `solventc` from the project's GitHub releases: no Rust, no build.
#
#     curl -fsSL https://raw.githubusercontent.com/bmander/geomsolver/main/scripts/install-solventc.sh | sh
#
# SOLVENTC_VERSION picks a tag (default: the latest release); SOLVENTC_DIR where the binary goes
# (default: ~/.local/bin).  The archive is checked against the release's SHA256SUMS, and the
# primer, the language as the binary accepts it, is left beside the binary as solvent-primer.md.
set -eu

repo=bmander/geomsolver
version=${SOLVENTC_VERSION:-latest}
dir=${SOLVENTC_DIR:-$HOME/.local/bin}

case "$(uname -s)" in
  Linux) os=unknown-linux-musl ;;
  Darwin) os=apple-darwin ;;
  *) echo "install-solventc: no prebuilt solventc for $(uname -s); build it with make solventc" >&2
     exit 1 ;;
esac
case "$(uname -m)" in
  x86_64|amd64) arch=x86_64 ;;
  aarch64|arm64) arch=aarch64 ;;
  *) echo "install-solventc: no prebuilt solventc for $(uname -m); build it with make solventc" >&2
     exit 1 ;;
esac
name=solventc-$arch-$os

if [ "$version" = latest ]; then base=https://github.com/$repo/releases/latest/download
else base=https://github.com/$repo/releases/download/$version; fi

tmp=$(mktemp -d)
trap 'rm -rf "$tmp"' EXIT
curl -fsSL "$base/$name.tar.gz" -o "$tmp/$name.tar.gz"
curl -fsSL "$base/SHA256SUMS" -o "$tmp/SHA256SUMS"
(cd "$tmp" && grep " $name.tar.gz\$" SHA256SUMS > want &&
  if command -v sha256sum > /dev/null; then sha256sum -c want > /dev/null 2>&1
  else shasum -a 256 -c want > /dev/null 2>&1; fi) ||
  { echo "install-solventc: $name.tar.gz does not match its SHA256SUMS" >&2; exit 1; }
tar xzf "$tmp/$name.tar.gz" -C "$tmp"

mkdir -p "$dir"
cp "$tmp/$name/solventc" "$tmp/$name/solvent-primer.md" "$dir/"
chmod +x "$dir/solventc"
echo "installed $("$dir/solventc" --version) to $dir/solventc"
case ":$PATH:" in *":$dir:"*) ;; *) echo "($dir is not on PATH)" ;; esac
