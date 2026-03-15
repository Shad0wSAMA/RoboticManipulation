import json

def adjust_z_height(z):
    """Adjust z from old range (0.07-0.09) to new range (0.068-0.073)"""
    old_min = 0.07
    old_max = 0.09
    new_min = 0.068
    new_max = 0.073

    # Normalize to 0-1
    normalized = (z - old_min) / (old_max - old_min)
    # Scale to new range
    new_z = new_min + normalized * (new_max - new_min)
    return round(new_z, 4)

def main():
    with open('strokes1.json', 'r') as f:
        data = json.load(f)

    # Find current min and max z
    all_z = []
    for stroke in data['strokes']:
        for point in stroke['points']:
            all_z.append(point[2])
    
    current_min = min(all_z)
    current_max = max(all_z)
    print(f"Current z range: {current_min} to {current_max}")

    target_min = 0.068
    target_max = 0.073

    for stroke in data['strokes']:
        for point in stroke['points']:
            # Normalize to 0-1
            if current_max > current_min:
                normalized = (point[2] - current_min) / (current_max - current_min)
            else:
                normalized = 0.5  # if all same, set to middle
            # Scale to target range
            point[2] = target_min + normalized * (target_max - target_min)
            point[2] = round(point[2], 4)

    with open('strokes1.json', 'w') as f:
        json.dump(data, f, indent=2)

    print(f"Z-heights adjusted to range {target_min} - {target_max}")
    print(f"New first point z: {data['strokes'][0]['points'][0][2]}")

if __name__ == "__main__":
    main()