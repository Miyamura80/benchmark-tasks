"""Evaluate a kitchen-scale collection test using measured masses and times."""
import argparse
import csv
import math
import statistics
from pathlib import Path

def assess(rows):
    groups = {stage: [] for stage in ('straight', 'kinked', 'released')}
    for row in rows:
        stage = row['stage']
        mass, seconds = float(row['net_water_g']), float(row['seconds'])
        if stage not in groups or not math.isfinite(mass) or not math.isfinite(seconds) or mass < 0 or seconds < 5:
            raise ValueError('Use a named stage, a nonnegative net mass and at least five seconds per observation')
        groups[stage].append(mass / seconds)
    if any(len(samples) < 3 for samples in groups.values()):
        raise ValueError('At least three observations are needed in each stage')
    reference = statistics.median(groups['straight'])
    if reference <= 0:
        raise ValueError('Straight reference must have measurable discharge')
    kinked = max(groups['kinked']) / reference
    released = min(groups['released']) / reference
    return {'straight_reference_g_s': reference, 'highest_kinked_fraction': kinked,
            'lowest_released_fraction': released, 'passes': kinked <= .20 and released >= .80}

if __name__ == '__main__':
    cli = argparse.ArgumentParser(description=__doc__)
    cli.add_argument('csv_file', type=Path)
    with cli.parse_args().csv_file.open() as source:
        import json
        print(json.dumps(assess(csv.DictReader(source)),indent=2))
