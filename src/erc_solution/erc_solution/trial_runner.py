#!/usr/bin/env python3
import subprocess
import time
import json
from datetime import datetime
import os

class TrialRunner:
    def __init__(self):
        self.results = []
        self.output_dir = '/erc_images'
        os.makedirs(self.output_dir, exist_ok=True)
        
    def run_trial(self, num_books=10):
        print(f'\nSTARTING {num_books}-BOOK TRIAL\n')
        start_time = time.time()
        
        for book_num in range(1, num_books + 1):
            print(f'[BOOK {book_num}/{num_books}] Starting grasp...')
            result = subprocess.run(
                ['ros2', 'run', 'erc_solution', 'grasp_controller'],
                capture_output=True,
                text=True,
                timeout=120
            )
            success = result.returncode == 0
            status = 'SUCCESS' if success else 'FAILED'
            print(f'[BOOK {book_num}] {status}')
            self.results.append({'book': book_num, 'success': success})
            time.sleep(2)
        
        elapsed = time.time() - start_time
        successful = sum(1 for r in self.results if r['success'])
        print(f'\nTRIAL COMPLETE: {successful}/{num_books} SUCCESS ({successful/num_books*100:.0f}%)')
        print(f'Total time: {elapsed:.0f}s\n')

if __name__ == '__main__':
    runner = TrialRunner()
    runner.run_trial(num_books=10)
