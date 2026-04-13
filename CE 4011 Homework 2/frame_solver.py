"""
Module: frame_solver
Purpose: Object-Oriented structural hierarchy and analysis pipeline with strict input validation.
"""
from matrix_library import Matrix, SparseMatrix

# --- INDEPENDENT CLASSES ---
class Material:
    def __init__(self, mat_id, E):
        if E <= 0: raise ValueError(f"Material {mat_id}: E must be positive. Received {E}")
        self.mat_id = mat_id
        self.E = E

class Section:
    def __init__(self, sec_id, A, I):
        if A <= 0: raise ValueError(f"Section {sec_id}: A must be positive. Received {A}")
        if I <= 0: raise ValueError(f"Section {sec_id}: I must be positive. Received {I}")
        self.sec_id = sec_id
        self.A = A
        self.I = I

# --- SUPPORTS & LOADS ---
class Support:
    def __init__(self):
        self.rx, self.ry, self.rz = 0, 0, 0

class Fixed(Support):
    def __init__(self): super().__init__(); self.rx, self.ry, self.rz = 1, 1, 1

class Pin(Support):
    def __init__(self): super().__init__(); self.rx, self.ry, self.rz = 1, 1, 0

class Roller(Support):
    def __init__(self): super().__init__(); self.rx, self.ry, self.rz = 0, 1, 0

class FixedRoller(Support):
    def __init__(self): super().__init__(); self.rx, self.ry, self.rz = 0, 1, 1

class Load:
    def __init__(self, load_id): self.load_id = load_id

class NodalLoad(Load):
    def __init__(self, load_id, fx=0.0, fy=0.0, mz=0.0):
        if fx == 0.0 and fy == 0.0 and mz == 0.0:
            raise ValueError(f"NodalLoad {load_id}: Must have non-zero force.")
        super().__init__(load_id)
        self.fx, self.fy, self.mz = fx, fy, mz

class MemberLoad(Load):
    def __init__(self, load_id): super().__init__(load_id)

class PointLoad(MemberLoad):
    def __init__(self, load_id, magnitude, location_ratio):
        if magnitude == 0.0: raise ValueError("Magnitude cannot be zero.")
        if not (0.0 <= location_ratio <= 1.0): raise ValueError("Location ratio out of bounds.")
        super().__init__(load_id)
        self.magnitude, self.location_ratio = magnitude, location_ratio

class UniformlyDistributedLoad(MemberLoad):
    def __init__(self, load_id, magnitude):
        if magnitude == 0.0: raise ValueError("Magnitude cannot be zero.")
        super().__init__(load_id)
        self.magnitude = magnitude

# --- NODES & ELEMENTS ---
class Node:
    def __init__(self, node_id, x, y):
        self.node_id, self.x, self.y = node_id, x, y
        self.support = None
        self.nodal_loads, self.connected_elements = [], []

    def assign_support(self, support): self.support = support
    def assign_load(self, load):
        if isinstance(load, NodalLoad): self.nodal_loads.append(load)

class Element:
    def __init__(self, elem_id): self.elem_id = elem_id

class Spring(Element):
    def __init__(self, elem_id, node, stiffness):
        if stiffness <= 0: raise ValueError("Stiffness must be positive.")
        super().__init__(elem_id)
        self.node, self.stiffness = node, stiffness
        node.connected_elements.append(self)

class Member(Element):
    def __init__(self, elem_id, start_node, end_node, material, section):
        if start_node.node_id == end_node.node_id: raise ValueError("Identical nodes.")
        if start_node.x == end_node.x and start_node.y == end_node.y: raise ValueError("Zero length.")
        super().__init__(elem_id)
        self.start_node, self.end_node = start_node, end_node
        self.material, self.section = material, section
        self.member_loads = []
        start_node.connected_elements.append(self)
        end_node.connected_elements.append(self)

    def assign_load(self, load):
        if isinstance(load, MemberLoad): self.member_loads.append(load)
        
    def get_length_and_angles(self):
        dx, dy = self.end_node.x - self.start_node.x, self.end_node.y - self.start_node.y
        L = (dx**2 + dy**2)**0.5
        return L, dx/L, dy/L

    def get_rotation_matrix(self):
        _, c, s = self.get_length_and_angles()
        R = Matrix(6, 6)
        R.set_val(0, 0, c); R.set_val(1, 1, c); R.set_val(3, 3, c); R.set_val(4, 4, c)
        R.set_val(0, 1, s); R.set_val(3, 4, s)
        R.set_val(1, 0, -s); R.set_val(4, 3, -s)
        R.set_val(2, 2, 1); R.set_val(5, 5, 1)
        return R

class TrussElement(Member):
    def __init__(self, elem_id, start_node, end_node, material, section):
        super().__init__(elem_id, start_node, end_node, material, section)

    def get_local_stiffness(self):
        # Trusses ONLY carry axial loads. Rotational and shear stiffnesses are 0.
        L, _, _ = self.get_length_and_angles()
        EA_L = (self.material.E * self.section.A) / L
        k_loc = Matrix(6, 6)
        k_loc.set_val(0, 0, EA_L); k_loc.set_val(3, 3, EA_L)
        k_loc.set_val(0, 3, -EA_L); k_loc.set_val(3, 0, -EA_L)
        return k_loc

class FrameElement(Member):
    def __init__(self, elem_id, start_node, end_node, material, section, release_start=False, release_end=False):
        super().__init__(elem_id, start_node, end_node, material, section)
        self.release_start = release_start
        self.release_end = release_end

    def get_length_and_angles(self):
        dx = self.end_node.x - self.start_node.x
        dy = self.end_node.y - self.start_node.y
        L = (dx**2 + dy**2)**0.5
        return L, dx/L, dy/L

    def get_local_stiffness(self):
        L, _, _ = self.get_length_and_angles()
        E, A, I = self.material.E, self.section.A, self.section.I
        k_loc = Matrix(6, 6)
        
        # Axial stiffness is always the same
        EA_L = E * A / L
        k_loc.set_val(0, 0, EA_L); k_loc.set_val(3, 3, EA_L)
        k_loc.set_val(0, 3, -EA_L); k_loc.set_val(3, 0, -EA_L)

        # Bending stiffness depends on releases
        if not self.release_start and not self.release_end: # Fixed-Fixed
            k_loc.set_val(1, 1, 12*E*I/(L**3)); k_loc.set_val(4, 4, 12*E*I/(L**3))
            k_loc.set_val(1, 4, -12*E*I/(L**3)); k_loc.set_val(4, 1, -12*E*I/(L**3))
            k_loc.set_val(1, 2, 6*E*I/(L**2)); k_loc.set_val(2, 1, 6*E*I/(L**2))
            k_loc.set_val(1, 5, 6*E*I/(L**2)); k_loc.set_val(5, 1, 6*E*I/(L**2))
            k_loc.set_val(4, 2, -6*E*I/(L**2)); k_loc.set_val(2, 4, -6*E*I/(L**2))
            k_loc.set_val(4, 5, -6*E*I/(L**2)); k_loc.set_val(5, 4, -6*E*I/(L**2))
            k_loc.set_val(2, 2, 4*E*I/L); k_loc.set_val(5, 5, 4*E*I/L)
            k_loc.set_val(2, 5, 2*E*I/L); k_loc.set_val(5, 2, 2*E*I/L)
            
        elif self.release_start and not self.release_end: # Pin-Fixed
            k_loc.set_val(1, 1, 3*E*I/(L**3)); k_loc.set_val(4, 4, 3*E*I/(L**3))
            k_loc.set_val(1, 4, -3*E*I/(L**3)); k_loc.set_val(4, 1, -3*E*I/(L**3))
            k_loc.set_val(1, 5, 3*E*I/(L**2)); k_loc.set_val(5, 1, 3*E*I/(L**2))
            k_loc.set_val(4, 5, -3*E*I/(L**2)); k_loc.set_val(5, 4, -3*E*I/(L**2))
            k_loc.set_val(5, 5, 3*E*I/L)
            # Row 2 and Col 2 remain 0
            
        elif not self.release_start and self.release_end: # Fixed-Pin
            k_loc.set_val(1, 1, 3*E*I/(L**3)); k_loc.set_val(4, 4, 3*E*I/(L**3))
            k_loc.set_val(1, 4, -3*E*I/(L**3)); k_loc.set_val(4, 1, -3*E*I/(L**3))
            k_loc.set_val(1, 2, 3*E*I/(L**2)); k_loc.set_val(2, 1, 3*E*I/(L**2))
            k_loc.set_val(4, 2, -3*E*I/(L**2)); k_loc.set_val(2, 4, -3*E*I/(L**2))
            k_loc.set_val(2, 2, 3*E*I/L)
            # Row 5 and Col 5 remain 0
            
        # If both are released (Pin-Pin), bending stiffness is 0, acting like a truss.
        return k_loc

    def get_rotation_matrix(self):
        _, c, s = self.get_length_and_angles()
        R = Matrix(6, 6)
        R.set_val(0, 0, c); R.set_val(1, 1, c); R.set_val(3, 3, c); R.set_val(4, 4, c)
        R.set_val(0, 1, s); R.set_val(3, 4, s)
        R.set_val(1, 0, -s); R.set_val(4, 3, -s)
        R.set_val(2, 2, 1); R.set_val(5, 5, 1)
        return R

    def get_fixed_end_forces(self):
        """Calculates FEF vector [Fx1, Fy1, M1, Fx2, Fy2, M2] for member loads."""
        L, _, _ = self.get_length_and_angles()
        fef = [0.0] * 6
        
        for load in self.member_loads:
            if isinstance(load, UniformlyDistributedLoad):
                w = load.magnitude
                if not self.release_start and not self.release_end:
                    fef[1] += w * L / 2; fef[2] += (w * L**2) / 12
                    fef[4] += w * L / 2; fef[5] -= (w * L**2) / 12
                elif self.release_start and not self.release_end:
                    fef[1] += 3 * w * L / 8; fef[4] += 5 * w * L / 8
                    fef[5] -= (w * L**2) / 8
                elif not self.release_start and self.release_end:
                    fef[1] += 5 * w * L / 8; fef[4] += 3 * w * L / 8
                    fef[2] += (w * L**2) / 8
            
            elif isinstance(load, PointLoad):
                P = load.magnitude
                a = load.location_ratio * L
                b = L - a
                if not self.release_start and not self.release_end:
                    fef[1] += P * (b**2) * (3*a + b) / (L**3)
                    fef[2] += P * a * (b**2) / (L**2)
                    fef[4] += P * (a**2) * (a + 3*b) / (L**3)
                    fef[5] -= P * (a**2) * b / (L**2)
                # Note: Add Pin-Fixed/Fixed-Pin formulas here if needed for Point loads
        return fef


# --- STRUCTURAL MODEL ---
class StructuralModel:
    def __init__(self):
        self.materials, self.sections, self.nodes, self.elements = {}, {}, {}, {}
        self.equation_map = {}
        self.num_equations = 0
        self.K_global_struct = None
        self.load_vector, self.displacements = [], []
        self.member_forces = {}

    def validate_model(self):
        if not self.nodes: raise ValueError("Model contains no nodes.")
        if not self.elements: raise ValueError("Model contains no elements.")

        coords = set()
        for n_id, node in self.nodes.items():
            coord = (node.x, node.y)
            if coord in coords: raise ValueError(f"Node {n_id} overlaps an existing node.")
            coords.add(coord)
            if len(node.connected_elements) == 0:
                raise ValueError(f"Node {n_id} is floating.")

        total_restraints = sum(n.support.rx + n.support.ry + n.support.rz for n in self.nodes.values() if n.support)
        if total_restraints < 3:
            raise ValueError("Kinematically unstable: Requires at least 3 restrained degrees of freedom.")

        visited_nodes = set()
        stack = [next(iter(self.nodes.values()))]
        while stack:
            curr_node = stack.pop()
            if curr_node.node_id not in visited_nodes:
                visited_nodes.add(curr_node.node_id)
                for elem in curr_node.connected_elements:
                    if hasattr(elem, 'start_node'):
                        adj = elem.end_node if elem.start_node == curr_node else elem.start_node
                        stack.append(adj)
        if len(visited_nodes) < len(self.nodes):
            raise ValueError("Disconnected sub-structures (floating members) detected.")

    def process_equations(self):
        self.validate_model()
        self.equation_map = {n_id: [0, 0, 0] for n_id in self.nodes.keys()}
        
        for n_id, node in self.nodes.items():
            rx, ry, rz = 0, 0, 0
            if node.support:
                rx, ry, rz = node.support.rx, node.support.ry, node.support.rz
            
            # LOCAL INSTABILITY FIX: If a node is free to rotate but ONLY connects to 
            # trusses, we must lock its rotation DOF to prevent a singular matrix.
            if rz == 0 and len(node.connected_elements) > 0:
                only_trusses = all(isinstance(e, TrussElement) for e in node.connected_elements)
                if only_trusses:
                    rz = 1 
                    
            self.equation_map[n_id] = [rx, ry, rz]
            
        eq_num = 1
        for n_id in sorted(self.nodes.keys()):
            for dof in range(3):
                if self.equation_map[n_id][dof] == 0:  
                    self.equation_map[n_id][dof] = eq_num
                    eq_num += 1
                else:  
                    self.equation_map[n_id][dof] = 0
        self.num_equations = eq_num - 1

    def assemble_matrices(self):
        self.K_global_struct = SparseMatrix(self.num_equations)
        for elem_id, elem in self.elements.items():
            if isinstance(elem, Member): # Applies to both Truss and Frame elements now
                k_loc = elem.get_local_stiffness()
                R = elem.get_rotation_matrix()
                k_glob = R.transpose().multiply(k_loc).multiply(R)
                G = self.equation_map[elem.start_node.node_id] + self.equation_map[elem.end_node.node_id]
                for p in range(6):
                    for q in range(6):
                        P, Q = G[p], G[q]
                        if P != 0 and Q != 0:  
                            self.K_global_struct.add_val(P - 1, Q - 1, k_glob.get_val(p, q))

    def solve_system(self):
        self.load_vector = [0.0] * self.num_equations
        
        # 1. Apply Nodal Loads
        for n_id, node in self.nodes.items():
            for load in node.nodal_loads:
                dofs = self.equation_map[n_id] 
                forces = [load.fx, load.fy, load.mz]
                for q in range(3):
                    if dofs[q] != 0: self.load_vector[dofs[q] - 1] += forces[q]
                    
        # 2. Apply Equivalent Nodal Loads from Member Loads (Subtract FEF)
        for elem in self.elements.values():
            if isinstance(elem, FrameElement) and elem.member_loads:
                fef_local = elem.get_fixed_end_forces()
                # Convert to global ENL: ENL_global = R^T * (-FEF_local)
                enl_local = [-f for f in fef_local]
                enl_global = elem.get_rotation_matrix().transpose().multiply(enl_local)
                
                G = self.equation_map[elem.start_node.node_id] + self.equation_map[elem.end_node.node_id]
                for q in range(6):
                    if G[q] != 0: self.load_vector[G[q] - 1] += enl_global[q]
        
        try:
            self.displacements = self.K_global_struct.solve(self.load_vector)
        except ValueError as e:
            if "Zero pivot" in str(e):
                raise ValueError("Structural validation failed: Kinematic instability or collapse mechanism detected.")
            else: raise e

    def calculate_internal_forces(self):
        for elem_id, elem in self.elements.items():
            if isinstance(elem, Member):
                G = self.equation_map[elem.start_node.node_id] + self.equation_map[elem.end_node.node_id]
                d_global = [0.0 if G[i] == 0 else self.displacements[G[i] - 1] for i in range(6)]
                
                R = elem.get_rotation_matrix()
                k_loc = elem.get_local_stiffness()
                d_local = R.multiply(d_global)
                
                # f = k * d + FEF
                forces = k_loc.multiply(d_local)
                if isinstance(elem, FrameElement):
                    fef = elem.get_fixed_end_forces()
                    forces = [forces[i] + fef[i] for i in range(6)]
                    
                self.member_forces[elem_id] = forces

    def calculate_reactions(self):
        """Calculates global support reactions by summing member end forces at restrained nodes."""
        self.reactions = {n_id: [0.0, 0.0, 0.0] for n_id, n in self.nodes.items() if n.support}
        
        for elem_id, elem in self.elements.items():
            if isinstance(elem, Member):
                # Convert final local forces back to global coordinates
                f_global = elem.get_rotation_matrix().transpose().multiply(self.member_forces[elem_id])
                
                sn_id = elem.start_node.node_id
                en_id = elem.end_node.node_id
                
                if sn_id in self.reactions:
                    self.reactions[sn_id][0] += f_global[0]
                    self.reactions[sn_id][1] += f_global[1]
                    self.reactions[sn_id][2] += f_global[2]
                    
                if en_id in self.reactions:
                    self.reactions[en_id][0] += f_global[3]
                    self.reactions[en_id][1] += f_global[4]
                    self.reactions[en_id][2] += f_global[5]