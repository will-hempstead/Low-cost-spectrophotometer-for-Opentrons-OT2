import sys
from time import sleep_ms
from machine import I2C, Pin
from as7341 import *
import os

FILENAME = "ph_sensor_auto.csv"   # change per run if you want

# ----------------- Hardware setup -----------------
led = Pin(3, Pin.OUT)

i2c = I2C(0, scl=Pin(1), sda=Pin(0), freq=100000)
sensor = AS7341(i2c)
if not sensor.isconnected():
    print("Failed to contact AS7341, terminating")
    sys.exit(1)

sensor.set_measure_mode(AS7341_MODE_SPM)
sensor.set_atime(100)
sensor.set_astep(2000)
sensor.set_again(4)

CHANNELS = ["f1","f2","f3","f4","f5","f6","f7","f8","clr","nir"]
BANDS = ["f1","f2","f3","f4","f5","f6","f7","f8"]

fmt = {
    "f1" : 'F1 (405-425nm): {:d}',
    "f2" : 'F2 (435-455nm): {:d}',
    "f3" : 'F3 (470-490nm): {:d}',
    "f4" : 'F4 (505-525nm): {:d}',
    "f5" : 'F5 (545-565nm): {:d}',
    "f6" : 'F6 (580-600nm): {:d}',
    "f7" : 'F7 (620-640nm): {:d}',
    "f8" : 'F8 (670-690nm): {:d}',
    "clr": 'Clear: {:d}',
    "nir": 'NIR: {:d}'
}

def p(f, v):
    print(fmt[f].format(v))

# ----------------- Core sensor read -----------------
def measurement(verbose=True):
    if verbose:
        print("Taking measurement . . .")

    sensor.start_measure("F1F4CN")
    f1,f2,f3,f4,clr,nir = sensor.get_spectral_data()
    sensor.start_measure("F5F8CN")
    f5,f6,f7,f8,clr,nir = sensor.get_spectral_data()

    vals = [f1,f2,f3,f4,f5,f6,f7,f8,clr,nir]

    if verbose:
        p("f1", f1); p("f2", f2); p("f3", f3); p("f4", f4)
        p("f5", f5); p("f6", f6); p("f7", f7); p("f8", f8)
        p("clr", clr); p("nir", nir)
        print("------------------------")

    sleep_ms(50)
    return vals

def mean_of_replicates(n=5, led_current=20, settle_ms=100, gap_ms=100, verbose=False):
    sensor.set_led_current(led_current)
    sleep_ms(settle_ms)

    sums = [0]*len(CHANNELS)
    for i in range(n):
        vals = measurement(verbose=verbose)
        for j in range(len(sums)):
            sums[j] += vals[j]
        sleep_ms(gap_ms)

    # turn off LED
    sensor.set_led_current(0)
    led.off()

    means = [s//n for s in sums]   # integer mean (fast, stable on MCU)
    return means

# ----------------- Normalization & features -----------------
def normalize_against_clr(vals):
    # vals = [f1..f8, clr, nir]
    clr = vals[8]
    if clr <= 0:
        # avoid div by zero; return zeros (or you could return None and skip)
        return {b: 0.0 for b in BANDS}, 0
    norm = {}
    for idx, b in enumerate(BANDS):
        norm[b] = vals[idx] / clr
    return norm, clr

def ndi(pos_val, neg_val):
    denom = (pos_val + neg_val)
    if denom == 0:
        return 0.0
    return (pos_val - neg_val) / denom

# ----------------- Stats helpers (no numpy) -----------------
def pearson_r(x, y):
    # x,y are lists of floats
    n = len(x)
    if n < 2:
        return 0.0
    mx = sum(x)/n
    my = sum(y)/n
    num = 0.0
    dx2 = 0.0
    dy2 = 0.0
    for i in range(n):
        dx = x[i] - mx
        dy = y[i] - my
        num += dx*dy
        dx2 += dx*dx
        dy2 += dy*dy
    if dx2 == 0 or dy2 == 0:
        return 0.0
    return num / ((dx2**0.5) * (dy2**0.5))

def solve_3x3(A, b):
    # Gaussian elimination for 3x3
    # A is 3x3 list, b is length-3 list
    M = [
        [float(A[0][0]), float(A[0][1]), float(A[0][2]), float(b[0])],
        [float(A[1][0]), float(A[1][1]), float(A[1][2]), float(b[1])],
        [float(A[2][0]), float(A[2][1]), float(A[2][2]), float(b[2])]
    ]

    # Forward elimination
    for col in range(3):
        # pivot
        pivot = col
        for r in range(col+1, 3):
            if abs(M[r][col]) > abs(M[pivot][col]):
                pivot = r
        if abs(M[pivot][col]) < 1e-12:
            return (0.0, 0.0, 0.0)  # singular-ish fallback
        if pivot != col:
            M[col], M[pivot] = M[pivot], M[col]

        # normalize pivot row
        div = M[col][col]
        for c in range(col, 4):
            M[col][c] /= div

        # eliminate below
        for r in range(col+1, 3):
            factor = M[r][col]
            for c in range(col, 4):
                M[r][c] -= factor * M[col][c]

    # Back substitution
    x2 = M[2][3]
    x1 = M[1][3] - M[1][2]*x2
    x0 = M[0][3] - M[0][2]*x2 - M[0][1]*x1
    return (x0, x1, x2)  # corresponds to [a, b, c] in ax^2 + bx + c

def polyfit_deg2(x, y):
    # Fit y = a x^2 + b x + c via normal equations
    n = len(x)
    Sx  = sum(x)
    Sx2 = sum([xi*xi for xi in x])
    Sx3 = sum([xi*xi*xi for xi in x])
    Sx4 = sum([xi*xi*xi*xi for xi in x])

    Sy   = sum(y)
    Sxy  = sum([x[i]*y[i] for i in range(n)])
    Sx2y = sum([(x[i]*x[i])*y[i] for i in range(n)])

    A = [
        [Sx4, Sx3, Sx2],
        [Sx3, Sx2, Sx ],
        [Sx2, Sx , n  ]
    ]
    b = [Sx2y, Sxy, Sy]
    a, bb, c = solve_3x3(A, b)
    return a, bb, c

def polyval_deg2(a, b, c, x):
    return a*x*x + b*x + c

# ----------------- CSV helpers -----------------
def ensure_header():
    # Write header if file doesn't exist or is empty
    try:
        stat = os.stat(FILENAME)
        if stat[6] > 0:
            return
    except:
        pass

    header = (
        "mode,ph_label,"
        + ",".join(CHANNELS) + ","
        + ",".join(["n_"+b for b in BANDS]) + ","
        + "pos_band,neg_band,index,pred_ph\n"
    )
    with open(FILENAME, "a") as f:
        f.write(header)

def log_row(mode, ph_label, raw_vals, norm_dict, pos_band, neg_band, index_val, pred_ph):
    raw_str = ",".join([str(v) for v in raw_vals])
    norm_str = ",".join([("{:.6f}".format(norm_dict[b])) for b in BANDS])

    line = (
        "{},{},"
        "{},"  # raw
        "{},"  # norm
        "{},{},{:.6f},{:.4f}\n"
    ).format(mode, ph_label, raw_str, norm_str, pos_band, neg_band, index_val, pred_ph)

    with open(FILENAME, "a") as f:
        f.write(line)

# ----------------- Main workflow -----------------
def run_calibration():
    print("\n=== Calibration setup ===")
    n_solutions = int(input("How many calibration solutions? (e.g., 3): "))
    reps = int(input("How many replicates per solution? (e.g., 5): "))

    ph_list = []
    norm_by_band = {b: [] for b in BANDS}
    raw_means_store = []

    ensure_header()

    for s in range(n_solutions):
        ph = float(input("\nEnter solution pH (e.g., 4, 7, 9): "))
        input("Place sensor in solution and press Enter to measure...")

        led.on()
        mean_raw = mean_of_replicates(n=reps, led_current=20, settle_ms=150, gap_ms=120, verbose=False)
        raw_means_store.append(mean_raw)

        norm, clr = normalize_against_clr(mean_raw)
        if clr <= 0:
            print("WARNING: CLR was 0; normalization invalid. Try again with better light/position.")
            continue

        ph_list.append(ph)
        for b in BANDS:
            norm_by_band[b].append(norm[b])

        # log calibration mean row (pred_ph empty for now -> use 0.0)
        log_row(
            mode="cal_mean",
            ph_label=ph,
            raw_vals=mean_raw,
            norm_dict=norm,
            pos_band="",
            neg_band="",
            index_val=0.0,
            pred_ph=0.0
        )

        print("Stored mean of", reps, "replicates for pH =", ph)
        print("Mean CLR =", mean_raw[8])

    # --- Correlation analysis to pick best channels ---
    print("\n=== Band–pH correlation (using normalized bands) ===")
    best_pos = None
    best_neg = None
    best_pos_r = -999
    best_neg_r =  999

    for b in BANDS:
        r = pearson_r(norm_by_band[b], ph_list)
        print(b, "r =", "{:.4f}".format(r))
        if r > best_pos_r:
            best_pos_r = r
            best_pos = b
        if r < best_neg_r:
            best_neg_r = r
            best_neg = b

    print("\nStrongest positive:", best_pos, "r =", "{:.4f}".format(best_pos_r))
    print("Strongest negative:", best_neg, "r =", "{:.4f}".format(best_neg_r))

    # --- Build NDI and polynomial fit ---
    idx_list = []
    for i in range(len(ph_list)):
        posv = norm_by_band[best_pos][i]
        negv = norm_by_band[best_neg][i]
        idx_list.append(ndi(posv, negv))

    a, b, c = polyfit_deg2(idx_list, ph_list)
    print("\n=== Polynomial fit (pH = a*I^2 + b*I + c) ===")
    print("a =", a)
    print("b =", b)
    print("c =", c)

    # Save model summary as a line in CSV (optional)
    with open(FILENAME, "a") as f:
        f.write("model_summary,,a={},b={},c={},pos={},neg={}\n".format(a, b, c, best_pos, best_neg))

    return best_pos, best_neg, a, b, c

def run_unknown_loop(best_pos, best_neg, a, b, c):
    print("\n=== Unknown measurement loop ===")
    reps_unknown = int(input("Replicates for unknown measurement? (1 or 5): "))

    while True:
        x = int(input("\nEnter 1 to measure unknown, 0 to quit: "))
        if x == 0:
            led.off()
            sensor.set_led_current(0)
            print("Exiting.")
            sys.exit(0)

        label = input("Optional sample label (press Enter to skip): ")

        led.on()
        mean_raw = mean_of_replicates(n=reps_unknown, led_current=20, settle_ms=150, gap_ms=120, verbose=False)
        norm, clr = normalize_against_clr(mean_raw)
        if clr <= 0:
            print("WARNING: CLR was 0; can't normalize. Try again.")
            continue

        I = ndi(norm[best_pos], norm[best_neg])
        pred_ph = polyval_deg2(a, b, c, I)

        print("Selected bands:", best_pos, "(+)", "and", best_neg, "(-)")
        print("Index I =", I)
        print("Predicted pH =", pred_ph)

        log_row(
            mode="unknown_mean",
            ph_label=label,
            raw_vals=mean_raw,
            norm_dict=norm,
            pos_band=best_pos,
            neg_band=best_neg,
            index_val=I,
            pred_ph=pred_ph
        )

        led.off()

# ----------------- Run -----------------
def main():
    print("AS7341 pH calibration + automated analysis")
    best_pos, best_neg, a, b, c = run_calibration()
    run_unknown_loop(best_pos, best_neg, a, b, c)

main()
