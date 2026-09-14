import re
import sys

# python3 initialization/simplify_expansion.py costdump top_gcd_snippets/costdump/cost_iter1_x168000_y50400_s37.txt top_gcd_snippets/costdump/cost_iter1_x168000_y50400_s37_simplified.txt
# python3 initialization/simplify_expansion.py expdump top_gcd_snippets/costdump/exp_iter1_x168000_y50400_s37.txt top_gcd_snippets/costdump/exp_iter1_x168000_y50400_s37_simplified.txt


def process_costdump(input_filepath, output_filepath):
    """Parses costdump files, filtering for 'expanding' lines."""
    pattern = re.compile(
        r'expanding\s+(?P<x>\d+)\s+(?P<y>\d+)\s+(?P<z>\d+)'
        r'(?:\s+pt\s+\d+\s+\d+)?'
        r'\s+cost\s+(?P<cost>\d+)'
        r'\s+pathCost\s+(?P<path_cost>\d+)'
        r'\s+lastDir\s+(?P<last_dir>\S+)'
    )

    count = 0
    with open(input_filepath, 'r') as fin, open(output_filepath, 'w') as fout:
        for line in fin:
            line = line.strip()
            if line.startswith('expanding'):
                match = pattern.search(line)
                if match:
                    d = match.groupdict()
                    fout.write(
                        f"node: ({d['x']}, {d['y']}, {d['z']}) | cost: {d['cost']} | pathCost: {d['path_cost']} | lastDir: {d['last_dir']}\n"
                    )
                else:
                    fout.write(f"{line}\n")
                count += 1

    print(f"[costdump] Processed {count} expansion steps into '{output_filepath}'.")

def process_expdump(input_filepath, output_filepath):
    """Parses expdump files, tracking attempted expansions (VALID vs INVALID)."""
    test_pattern = re.compile(
        r'testing\s+(?P<x>\d+)\s+(?P<y>\d+)\s+(?P<z>\d+)\s+prev\s+A\*\s+node\s+dir\s+(?P<prev_dir>\S+)'
    )
    exp_pattern = re.compile(
        r'expanding\s+(?P<x>\d+)\s+(?P<y>\d+)\s+(?P<z>\d+)'
        r'(?:\s+pt\s+\d+\s+\d+)?'
        r'\s+cost\s+(?P<cost>\d+)'
        r'\s+pathCost\s+(?P<path_cost>\d+)'
        r'\s+lastDir\s+(?P<last_dir>\S+)'
    )

    pending_test = None
    count = 0

    with open(input_filepath, 'r') as fin, open(output_filepath, 'w') as fout:
        for line in fin:
            line = line.strip()
            if not line:
                continue

            # Line is an expansion attempt test (excluding nextWavefrontGrid)
            if line.startswith('testing') and 'nextWavefrontGrid' not in line:
                # Flush previous pending test as INVALID if it was never followed by 'expanding'
                if pending_test:
                    fout.write(
                        f"node: ({pending_test['x']}, {pending_test['y']}, {pending_test['z']}) | status: INVALID | prevDir: {pending_test['prev_dir']}\n"
                    )
                    count += 1

                match = test_pattern.search(line)
                if match:
                    pending_test = match.groupdict()

            # Line indicates successful expansion
            elif line.startswith('expanding'):
                if pending_test:
                    match = exp_pattern.search(line)
                    if match:
                        d = match.groupdict()
                        fout.write(
                            f"node: ({d['x']}, {d['y']}, {d['z']}) | status: VALID | cost: {d['cost']} | pathCost: {d['path_cost']} | lastDir: {d['last_dir']}\n"
                        )
                    else:
                        fout.write(f"node: ({pending_test['x']}, {pending_test['y']}, {pending_test['z']}) | status: VALID\n")
                    pending_test = None
                    count += 1

            # Any intermediate line means the pending test didn't expand
            else:
                if pending_test:
                    fout.write(
                        f"node: ({pending_test['x']}, {pending_test['y']}, {pending_test['z']}) | status: INVALID | prevDir: {pending_test['prev_dir']}\n"
                    )
                    pending_test = None
                    count += 1

        # EOF flush
        if pending_test:
            fout.write(
                f"node: ({pending_test['x']}, {pending_test['y']}, {pending_test['z']}) | status: INVALID | prevDir: {pending_test['prev_dir']}\n"
            )
            count += 1

    print(f"[expdump] Processed {count} expansion attempts into '{output_filepath}'.")

if __name__ == '__main__':
    if len(sys.argv) < 4:
        print("Usage: python simplify_logs.py <mode: costdump|expdump> <input_file> <output_file>")
        sys.exit(1)

    mode = sys.argv[1].lower()
    input_file = sys.argv[2]
    output_file = sys.argv[3]

    if mode == 'costdump':
        process_costdump(input_file, output_file)
    elif mode == 'expdump':
        process_expdump(input_file, output_file)
    else:
        print(f"Error: Unknown mode '{mode}'. Use 'costdump' or 'expdump'.")
        sys.exit(1)