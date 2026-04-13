"""
Module: main
Purpose: Entry point. Parses input, builds the object-oriented model, and runs the analysis.
"""
from frame_solver import (StructuralModel, Material, Section, Node, 
                          FrameElement, TrussElement, Spring, 
                          Pin, Roller, Fixed, FixedRoller, NodalLoad)

def parse_input_file(filepath, model):
    current_section = None
    with open(filepath, 'r') as file:
        for line in file:
            line = line.strip()
            if not line or line.startswith('#'): continue
            if line.startswith('[') and line.endswith(']'):
                current_section = line[1:-1].lower()
                continue
            
            parts = [p.strip() for p in line.split('/')]
            
            if current_section == 'materials':
                mat_id = int(parts[0])
                model.materials[mat_id] = Material(mat_id, float(parts[3]))
                model.sections[mat_id] = Section(mat_id, float(parts[1]), float(parts[2])) 

            elif current_section == 'nodes':
                n_id = int(parts[0])
                model.nodes[n_id] = Node(n_id, float(parts[1]), float(parts[2]))

            elif current_section == 'members':
                # Mem ID / Start / End / Mat ID / Type / Release Start (0/1) / Release End (0/1)
                mem_id = int(parts[0])
                start_n = model.nodes[int(parts[1])]
                end_n = model.nodes[int(parts[2])]
                mat = model.materials[int(parts[3])]
                sec = model.sections[int(parts[3])]
                
                mem_type = parts[4].strip().lower() if len(parts) > 4 else 'frame'
                rel_start = bool(int(parts[5])) if len(parts) > 5 else False
                rel_end = bool(int(parts[6])) if len(parts) > 6 else False
                
                if mem_type == 'truss':
                    model.elements[mem_id] = TrussElement(mem_id, start_n, end_n, mat, sec)
                else:
                    model.elements[mem_id] = FrameElement(mem_id, start_n, end_n, mat, sec, rel_start, rel_end)

            elif current_section == 'springs':
                model.elements[int(parts[0])] = Spring(int(parts[0]), model.nodes[int(parts[1])], float(parts[2]))

            elif current_section == 'supports':
                # Node ID / RX / RY / RZ  (Exactly as defined in the PDF)
                n_id, rx, ry, rz = int(parts[0]), int(parts[1]), int(parts[2]), int(parts[3])
                
                support_obj = None
                if rx == 1 and ry == 1 and rz == 1: support_obj = Fixed()
                elif rx == 1 and ry == 1 and rz == 0: support_obj = Pin()
                elif rx == 0 and ry == 1 and rz == 0: support_obj = Roller()
                elif rx == 0 and ry == 1 and rz == 1: support_obj = FixedRoller() 
                
                if support_obj: 
                    model.nodes[n_id].assign_support(support_obj)

            elif current_section == 'loads':
                load_obj = NodalLoad(int(parts[0]), float(parts[2]), float(parts[3]), float(parts[4]))
                model.nodes[int(parts[1])].assign_load(load_obj)

def write_text_report(model, out_filepath):
    with open(out_filepath, 'w') as f:
        f.write("=== STRUCTURAL ANALYSIS REPORT ===\n\n")
        
        f.write("--- EQUATION NUMBERING ---\n")
        for n_id in sorted(model.equation_map.keys()):
            dofs = model.equation_map[n_id]
            f.write(f"Node {n_id:2} | DX: {dofs[0]:2} | DY: {dofs[1]:2} | RZ: {dofs[2]:2}\n")
            
        f.write(f"\nTotal Equations: {model.num_equations}\n\n")
        
        f.write("--- STRUCTURAL DISPLACEMENTS ---\n")
        for i, d in enumerate(model.displacements):
            f.write(f"Eq {i+1:2}: {d:>12.4e}\n")
            
        f.write("\n--- MEMBER END FORCES (Local) ---\n")
        for mem_id, forces in model.member_forces.items():
            f.write(f"\nMember {mem_id}:\n")
            f.write(f"  Start -> Axial: {forces[0]:8.2f}, Shear: {forces[1]:8.2f}, Moment: {forces[2]:8.2f}\n")
            f.write(f"  End   -> Axial: {forces[3]:8.2f}, Shear: {forces[4]:8.2f}, Moment: {forces[5]:8.2f}\n")

        # --- NEW SECTION FOR REACTIONS ---
        f.write("\n--- SUPPORT REACTIONS (Global) ---\n")
        for n_id, reactions in model.reactions.items():
            node = model.nodes[n_id]
            # Only print values for DOFs that are actually restrained (otherwise it prints ~0.00 for free DOFs)
            rx = reactions[0] if node.support.rx == 1 else 0.00
            ry = reactions[1] if node.support.ry == 1 else 0.00
            rz = reactions[2] if node.support.rz == 1 else 0.00
            f.write(f"Node {n_id:2} | Fx: {rx:8.2f} | Fy: {ry:8.2f} | Mz: {rz:8.2f}\n")

if __name__ == "__main__":
    model = StructuralModel()
    
    parse_input_file('input.txt', model)
    model.process_equations()
    model.assemble_matrices()
    model.solve_system()
    model.calculate_internal_forces()
    
    # Trigger the new reaction calculation!
    model.calculate_reactions()
    
    write_text_report(model, 'output_report.txt')
    print("Analysis complete! Check 'output_report.txt' for the results and reactions.")