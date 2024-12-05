#!/usr/bin/env python3

import unittest
import sys
import os

def run_tests():
    """Run all unit tests"""
    # Get the directory containing this script
    tests_dir = os.path.dirname(os.path.abspath(__file__))
    
    # Add the project root to Python path
    project_root = os.path.dirname(tests_dir)
    sys.path.insert(0, project_root)
    
    # Discover and run tests
    loader = unittest.TestLoader()
    start_dir = os.path.join(tests_dir, 'unit')
    suite = loader.discover(start_dir, pattern='test_*.py')
    
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    
    return result.wasSuccessful()

if __name__ == '__main__':
    success = run_tests()
    sys.exit(0 if success else 1) 