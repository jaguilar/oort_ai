use oort_simulator::simulation::{Code, Simulation};
use oort_simulator::scenario::{self, Status};
use oort_compiler::Compiler;

fn main() {
    let scenario_name = "tutorial_lead";
    let src = std::fs::read_to_string("scratch/bundled.rs").expect("Failed to read scratch/bundled.rs");
    let mut compiler = Compiler::new();
    let wasm = compiler.compile(&src).expect("Failed to compile Wasm");
    
    let codes = vec![
        Code::Wasm(wasm),
        Code::Builtin("empty".to_string()),
    ];
    
    let mut sim = Simulation::new(scenario_name, 0, &codes);
    println!("Simulation started!");
    
    while sim.status() == Status::Running && sim.tick() < 2000 {
        sim.step();
        for error in &sim.events().errors {
            println!("[Tick {}] ERROR: {}", sim.tick(), error.msg);
        }
        for (ship_id, text) in &sim.events().debug_text {
            println!("[Tick {}] Ship {}: {}", sim.tick(), ship_id, text.trim());
        }
    }
    
    println!("Finished simulation. Status: {:?}", sim.status());
}
