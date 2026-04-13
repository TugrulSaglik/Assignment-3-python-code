"""
Module: test_frame_solver.py
Purpose: Comprehensive unit tests covering basic entities, physics, and deep system-level user errors.
"""

import unittest
from frame_solver import (Material, Section, Support, Fixed, Pin, Roller, FixedRoller,
                          NodalLoad, PointLoad, UniformlyDistributedLoad, Node, 
                          Spring, FrameElement, TrussElement, StructuralModel)
from matrix_library import Matrix

class TestInputValidation(unittest.TestCase):
    """Verifies that individual object constructors reject impossible inputs."""
    
    def test_invalid_material(self):
        with self.assertRaises(ValueError): Material(mat_id=1, E=0.0)
        with self.assertRaises(ValueError): Material(mat_id=2, E=-200000.0)

    def test_invalid_section(self):
        with self.assertRaises(ValueError): Section(sec_id=1, A=0.0, I=0.08)
        with self.assertRaises(ValueError): Section(sec_id=2, A=0.02, I=-0.08)

    def test_invalid_member_nodes(self):
        mat = Material(1, 200000)
        sec = Section(1, 0.02, 0.08)
        node1 = Node(1, 0.0, 0.0)
        node2 = Node(2, 0.0, 0.0) # Overlapping coordinates
        
        # Test supplying the same exact node object
        with self.assertRaises(ValueError):
            FrameElement(elem_id=1, start_node=node1, end_node=node1, material=mat, section=sec)
            
        # Test supplying different nodes but identical coordinates (Zero Length)
        with self.assertRaises(ValueError):
            FrameElement(elem_id=2, start_node=node1, end_node=node2, material=mat, section=sec)

    def test_invalid_spring(self):
        node1 = Node(1, 0.0, 0.0)
        with self.assertRaises(ValueError): Spring(elem_id=1, node=node1, stiffness=0.0)
        with self.assertRaises(ValueError): Spring(elem_id=2, node=node1, stiffness=-500.0)

    def test_invalid_loads(self):
        with self.assertRaises(ValueError): NodalLoad(load_id=1, fx=0.0, fy=0.0, mz=0.0)
        with self.assertRaises(ValueError): PointLoad(load_id=2, magnitude=0.0, location_ratio=0.5)
        with self.assertRaises(ValueError): UniformlyDistributedLoad(load_id=3, magnitude=0.0)
        with self.assertRaises(ValueError): PointLoad(load_id=4, magnitude=10.0, location_ratio=-0.1)
        with self.assertRaises(ValueError): PointLoad(load_id=5, magnitude=10.0, location_ratio=1.5)


class TestSystemLevelValidation(unittest.TestCase):
    """Verifies that the entire structural model is physically valid and connected before solving."""
    
    def setUp(self):
        self.model = StructuralModel()
        self.mat = Material(1, 200000.0)
        self.sec = Section(1, 0.02, 0.08)

    def test_floating_node(self):
        """Catches a node that was created but never connected to a member/spring."""
        n1 = Node(1, 0.0, 0.0)
        n1.assign_support(Fixed())
        self.model.nodes[1] = n1
        # No elements are added
        with self.assertRaisesRegex(ValueError, "Model contains no elements"):
            self.model.validate_model()
            
        # Add a second node that will be left floating
        n2 = Node(2, 5.0, 0.0)
        self.model.nodes[2] = n2
        
        # Create a valid element connecting n1 to a new dummy node 
        n_dummy = Node(99, 1.0, 1.0) 
        self.model.nodes[99] = n_dummy
        self.model.elements[1] = FrameElement(1, n1, n_dummy, self.mat, self.sec)
        
        # Node 2 is still floating (not connected to any elements)
        with self.assertRaisesRegex(ValueError, "is floating"):
            self.model.validate_model()

    def test_disconnected_substructures(self):
        """Catches 'floating members' where two parts of a frame do not connect."""
        n1 = Node(1, 0.0, 0.0)
        n2 = Node(2, 4.0, 0.0)
        n3 = Node(3, 10.0, 10.0)
        n4 = Node(4, 14.0, 10.0)
        
        n1.assign_support(Fixed())
        n3.assign_support(Fixed()) # Both have enough supports
        
        self.model.nodes.update({1: n1, 2: n2, 3: n3, 4: n4})
        
        # Element 1 connects 1-2. Element 2 connects 3-4. They do not share nodes.
        self.model.elements[1] = FrameElement(1, n1, n2, self.mat, self.sec)
        self.model.elements[2] = FrameElement(2, n3, n4, self.mat, self.sec)
        
        with self.assertRaisesRegex(ValueError, "Disconnected sub-structures"):
            self.model.validate_model()

    def test_kinematic_instability(self):
        """Catches a model that doesn't have enough boundary conditions to prevent rigid body motion."""
        n1 = Node(1, 0.0, 0.0)
        n2 = Node(2, 4.0, 0.0)
        
        # We only assign a Roller (1 restraint). A 2D frame needs at least 3 restraints.
        n1.assign_support(Roller()) 
        
        self.model.nodes.update({1: n1, 2: n2})
        self.model.elements[1] = FrameElement(1, n1, n2, self.mat, self.sec)
        
        with self.assertRaisesRegex(ValueError, "Kinematically unstable"):
            self.model.validate_model()

    def test_overlapping_nodes_in_system(self):
        """Catches two different Node IDs being assigned to the exact same space."""
        n1 = Node(1, 5.0, 5.0)
        n2 = Node(2, 5.0, 5.0)
        n3 = Node(3, 10.0, 5.0)
        
        n1.assign_support(Fixed())
        
        self.model.nodes.update({1: n1, 2: n2, 3: n3})
        self.model.elements[1] = FrameElement(1, n1, n3, self.mat, self.sec)
        self.model.elements[2] = FrameElement(2, n2, n3, self.mat, self.sec)
        
        with self.assertRaisesRegex(ValueError, "overlaps an existing node"):
            self.model.validate_model()


class TestStructuralModelPhysics(unittest.TestCase):
    def setUp(self):
        self.model = StructuralModel()
        mat = Material(1, 200000.0)
        sec = Section(1, 0.02, 0.08)
        self.model.materials[1] = mat
        self.model.sections[1] = sec
        
        n1 = Node(1, 0.0, 0.0)
        n2 = Node(2, 4.0, 0.0)
        n1.assign_support(Fixed())
        n2.assign_load(NodalLoad(1, fx=0.0, fy=-10.0, mz=0.0))
        
        self.model.nodes[1] = n1
        self.model.nodes[2] = n2
        self.model.elements[1] = FrameElement(1, n1, n2, mat, sec)

    def test_full_pipeline_success(self):
        """Ensures a valid model solves without throwing validation errors."""
        self.model.process_equations() # Validate is called automatically here
        self.model.assemble_matrices()
        self.model.solve_system()
        self.model.calculate_internal_forces()
        
        # Check standard results
        self.assertEqual(self.model.num_equations, 3)
        self.assertIn(1, self.model.member_forces)
        self.assertEqual(len(self.model.member_forces[1]), 6)


class TestAdvancedMechanics(unittest.TestCase):
    def setUp(self):
        self.model = StructuralModel()
        self.mat = Material(1, 200000.0)
        self.sec = Section(1, 0.02, 0.08)

    def test_pure_truss_node_auto_restraint(self):
        """Proves that a node connecting ONLY to trusses has its rotational DOF locked."""
        n1 = Node(1, 0.0, 0.0)
        n2 = Node(2, 4.0, 0.0)
        n1.assign_support(Pin())
        n2.assign_support(Roller())
        self.model.nodes.update({1: n1, 2: n2})
        
        # Connect with a Truss Element
        self.model.elements[1] = TrussElement(1, n1, n2, self.mat, self.sec)
        self.model.process_equations()
        
        # A Pin starts as [1, 1, 0] (RZ free). Truss auto-locks RZ to 1 -> [1, 1, 1].
        # Equation numbering turns restrained DOFs (1) into 0s. 
        # So the final equation_map for Node 1 should be [0, 0, 0].
        self.assertEqual(self.model.equation_map[1], [0, 0, 0])

    def test_global_instability_catch(self):
        """Proves the system catches a collapse mechanism (zero pivot)."""
        n1 = Node(1, 0.0, 0.0)
        n2 = Node(2, 4.0, 0.0)
        
        # We need at least 3 restraints to pass validate_model(), but arranged 
        # to allow a mechanism. FixedRoller (RY, RZ) + Roller (RY) = 3 restraints.
        # But neither restricts X, so the matrix will be singular in X.
        n1.assign_support(FixedRoller()) 
        n2.assign_support(Roller()) 
        
        self.model.nodes.update({1: n1, 2: n2})
        self.model.elements[1] = FrameElement(1, n1, n2, self.mat, self.sec)
        
        self.model.process_equations()
        self.model.assemble_matrices()
        
        # It should pass basic checks but fail during matrix solving due to the singularity
        with self.assertRaisesRegex(ValueError, "collapse mechanism"):
            self.model.solve_system()

    class TestRegressionSuite(unittest.TestCase):
        """
        Regression tests building full structural models programmatically to ensure 
        the entire analysis pipeline (Frame, Truss, and Mixed systems) works end-to-end.
        """
    def setUp(self):
        self.model = StructuralModel()
        self.mat = Material(1, 200000.0)
        self.sec = Section(1, 0.02, 0.08)

    def test_regression_frame(self):
        """Regression Test 1: Stable Portal Frame (Pure Frame Elements)"""
        n1 = Node(1, 0.0, 0.0); n1.assign_support(Pin())
        n2 = Node(2, 0.0, 4.0)
        n3 = Node(3, 5.0, 4.0)
        n4 = Node(4, 5.0, 0.0); n4.assign_support(Pin())
        
        # Apply 10 kN horizontal load to the top left corner
        n2.assign_load(NodalLoad(1, fx=10.0, fy=0.0, mz=0.0))
        
        self.model.nodes.update({1: n1, 2: n2, 3: n3, 4: n4})
        self.model.elements[1] = FrameElement(1, n1, n2, self.mat, self.sec)
        self.model.elements[2] = FrameElement(2, n2, n3, self.mat, self.sec)
        self.model.elements[3] = FrameElement(3, n3, n4, self.mat, self.sec)
        
        # Run full pipeline (Added calculate_internal_forces here!)
        self.model.process_equations()
        self.model.assemble_matrices()
        self.model.solve_system()
        self.model.calculate_internal_forces()  # <--- FIX IS HERE
        self.model.calculate_reactions()
        
        # VERIFICATION: Global Equilibrium (ΣFx = 0)
        sum_fx_reactions = self.model.reactions[1][0] + self.model.reactions[4][0]
        self.assertAlmostEqual(sum_fx_reactions, -10.0, places=4)

    def test_regression_truss(self):
        """Regression Test 2: 2-Member Apex Truss (Pure Truss Elements)"""
        n1 = Node(1, 0.0, 0.0); n1.assign_support(Pin())
        n2 = Node(2, 3.0, 4.0)
        n3 = Node(3, 6.0, 0.0); n3.assign_support(Pin())
        
        # Apply 10 kN horizontal and 15 kN downward at the apex
        n2.assign_load(NodalLoad(1, fx=10.0, fy=-15.0, mz=0.0))
        
        self.model.nodes.update({1: n1, 2: n2, 3: n3})
        self.model.elements[1] = TrussElement(1, n1, n2, self.mat, self.sec)
        self.model.elements[2] = TrussElement(2, n2, n3, self.mat, self.sec)
        
        # Run full pipeline (Added calculate_internal_forces here!)
        self.model.process_equations()
        self.model.assemble_matrices()
        self.model.solve_system()
        self.model.calculate_internal_forces()  # <--- FIX IS HERE
        self.model.calculate_reactions()
        
        # VERIFICATION 1: Truss Rotational DOF auto-locking feature worked.
        self.assertEqual(self.model.equation_map[2][2], 0)
        
        # VERIFICATION 2: Global Equilibrium (ΣFy = 0)
        sum_fy_reactions = self.model.reactions[1][1] + self.model.reactions[3][1]
        self.assertAlmostEqual(sum_fy_reactions, 15.0, places=4)

    def test_regression_mixed_frame_truss(self):
        """Regression Test 3: Mixed Structure (Fixed Frame Column + Diagonal Truss Brace)"""
        n1 = Node(1, 0.0, 0.0); n1.assign_support(Fixed())
        n2 = Node(2, 0.0, 4.0)
        n3 = Node(3, 4.0, 0.0); n3.assign_support(Pin())
        
        # Load pushing the column to the right
        n2.assign_load(NodalLoad(1, fx=25.0, fy=0.0, mz=0.0))
        
        self.model.nodes.update({1: n1, 2: n2, 3: n3})
        self.model.elements[1] = FrameElement(1, n1, n2, self.mat, self.sec) 
        self.model.elements[2] = TrussElement(2, n2, n3, self.mat, self.sec) 
        
        # Run full pipeline
        self.model.process_equations()
        self.model.assemble_matrices()
        self.model.solve_system()
        self.model.calculate_internal_forces()
        self.model.calculate_reactions() # Added this here just to be safe!
        
        # VERIFICATION
        self.assertTrue(len(self.model.displacements) > 0)
        self.assertIn(1, self.model.member_forces)
        self.assertIn(2, self.model.member_forces)

if __name__ == '__main__':
    unittest.main(verbosity=2)