use std::path::{Path, PathBuf};
use std::process::Command;

const GIT_DIRTY_STATUS_ARGS: &[&str] = &["status", "--porcelain"];
const BUILD_PROVENANCE_INPUTS: &[&str] = &[
    "src",
    "Cargo.toml",
    "Cargo.lock",
    "build.rs",
    ".cargo/config.toml",
];

fn main() {
    println!("cargo:rustc-link-arg-bins=--nmagic");
    println!("cargo:rustc-link-arg-bins=-Tlink.x");
    println!("cargo:rustc-link-arg-bins=-Tdefmt.x");

    for path in BUILD_PROVENANCE_INPUTS {
        println!("cargo:rerun-if-changed={path}");
    }
    emit_git_rerun_inputs();
    println!("cargo:rerun-if-env-changed=VESTA_SCAN_INTERVAL_SECS");

    let profile_v2 = std::env::var_os("CARGO_FEATURE_PROFILE_V2").is_some();
    let uart_training = std::env::var_os("CARGO_FEATURE_PROFILE_V2_UART").is_some();
    let radio_profile = profile_v2 && !uart_training;
    let default_interval = if uart_training { 15_u32 } else { 300_u32 };
    // The profile cadence variable must never change whether the deployed v1
    // image builds; v1 retains its independent fixed one-minute policy.
    let scan_interval = if profile_v2 {
        std::env::var("VESTA_SCAN_INTERVAL_SECS")
            .ok()
            .map(|value| {
                value
                    .parse::<u32>()
                    .expect("VESTA_SCAN_INTERVAL_SECS must be an unsigned integer")
            })
            .unwrap_or(default_interval)
    } else {
        default_interval
    };
    if profile_v2 {
        let minimum_interval = if radio_profile { 180 } else { 15 };
        assert!(
            scan_interval >= minimum_interval,
            "VESTA_SCAN_INTERVAL_SECS must be at least {minimum_interval} seconds for this build: UART needs the bounded collection window; radio profile-v2 is capped to protect the EU868 1% duty-cycle budget"
        );
        assert!(
            scan_interval <= u32::MAX / 1_000,
            "VESTA_SCAN_INTERVAL_SECS is too large for protocol milliseconds"
        );
    }
    println!("cargo:rustc-env=VESTA_SCAN_INTERVAL_SECS={scan_interval}");

    let build_id = Command::new("git")
        .args(["rev-parse", "--short=16", "HEAD"])
        .output()
        .ok()
        .filter(|output| output.status.success())
        .and_then(|output| String::from_utf8(output.stdout).ok())
        .map(|value| value.trim().to_owned())
        .filter(|value| value.len() == 16)
        .unwrap_or_default();
    println!("cargo:rustc-env=VESTA_BUILD_ID_HEX={build_id}");

    let dirty = Command::new("git")
        // Include untracked files: a new source module can affect the linked
        // image and must never be reported as a clean build.
        .args(GIT_DIRTY_STATUS_ARGS)
        .output()
        .ok()
        .filter(|output| output.status.success())
        .is_some_and(|output| !output.stdout.is_empty());
    println!("cargo:rustc-env=VESTA_BUILD_DIRTY={}", u8::from(dirty));
}

fn emit_git_rerun_inputs() {
    let Some(git_dir) = git_path(&["rev-parse", "--absolute-git-dir"]) else {
        return;
    };
    let common_dir = git_path(&["rev-parse", "--path-format=absolute", "--git-common-dir"])
        .unwrap_or_else(|| git_dir.clone());
    let symbolic_head = git_stdout(&["symbolic-ref", "-q", "HEAD"]);
    for path in git_rerun_inputs(&git_dir, &common_dir, symbolic_head.as_deref()) {
        println!("cargo:rerun-if-changed={}", path.display());
    }
}

fn git_path(arguments: &[&str]) -> Option<PathBuf> {
    git_stdout(arguments).map(PathBuf::from)
}

fn git_stdout(arguments: &[&str]) -> Option<String> {
    Command::new("git")
        .args(arguments)
        .output()
        .ok()
        .filter(|output| output.status.success())
        .and_then(|output| String::from_utf8(output.stdout).ok())
        .map(|value| value.trim().to_owned())
        .filter(|value| !value.is_empty())
}

fn git_rerun_inputs(
    git_dir: &Path,
    common_dir: &Path,
    symbolic_head: Option<&str>,
) -> Vec<PathBuf> {
    let mut inputs = vec![
        git_dir.join("HEAD"),
        git_dir.join("index"),
        common_dir.join("packed-refs"),
    ];
    if let Some(reference) = symbolic_head {
        inputs.push(common_dir.join(reference));
    }
    inputs
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn provenance_status_command_includes_untracked_files() {
        assert_eq!(GIT_DIRTY_STATUS_ARGS, ["status", "--porcelain"]);
        assert!(!GIT_DIRTY_STATUS_ARGS.contains(&"--untracked-files=no"));
    }

    #[test]
    fn provenance_reruns_for_source_and_build_input_changes() {
        assert!(BUILD_PROVENANCE_INPUTS.contains(&"src"));
        assert!(BUILD_PROVENANCE_INPUTS.contains(&"Cargo.toml"));
        assert!(BUILD_PROVENANCE_INPUTS.contains(&"Cargo.lock"));
        assert!(BUILD_PROVENANCE_INPUTS.contains(&"build.rs"));
        assert!(BUILD_PROVENANCE_INPUTS.contains(&".cargo/config.toml"));
    }

    #[test]
    fn provenance_watches_resolved_branch_and_worktree_metadata() {
        let inputs = git_rerun_inputs(
            Path::new("/repo/.git/worktrees/review"),
            Path::new("/repo/.git"),
            Some("refs/heads/profile-v2"),
        );
        assert!(inputs.contains(&PathBuf::from("/repo/.git/worktrees/review/HEAD")));
        assert!(inputs.contains(&PathBuf::from("/repo/.git/worktrees/review/index")));
        assert!(inputs.contains(&PathBuf::from("/repo/.git/packed-refs")));
        assert!(inputs.contains(&PathBuf::from("/repo/.git/refs/heads/profile-v2")));
    }
}
