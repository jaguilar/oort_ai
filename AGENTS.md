# General Rules

* Always run `cargo` with the `--offline` flag.

# Rust Style: Flat Control Flow
Invert conditions and return early to eliminate deep nesting.

- Use `let-else` instead of `if let`/`match` when the failure arm is short.
- Use `?` for immediate `Option`/`Result` propagation.
- Use `if !cond { return; }` guard clauses.

```rust
// ANTI-PATTERN (Do not do)
fn process(id: Option<u64>) {
    if let Some(uid) = id {
        if let Ok(u) = get(uid) { /* logic */ }
    }
}

// PREFERRED (Do this)
fn process(id: Option<u64>) {
    let Some(uid) = id else { return; };
    let Ok(u) = get(uid) else { return; };
    /* logic */
}