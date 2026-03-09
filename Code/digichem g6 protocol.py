from opentrons import protocol_api

metadata = {
    "protocolName": "Selective Photospec Scan with Return",
    "apiLevel": "2.21"
}

def run(protocol: protocol_api.ProtocolContext):

    # -----------------------------
    # Load Labware
    # -----------------------------

    tiprack = protocol.load_labware("photospec_tiprack", "1")
    plate = protocol.load_labware("hackspace_15_wellplate_7500ul", "2")

    # -----------------------------
    # Load Pipette
    # -----------------------------

    pipette = protocol.load_instrument(
        "p1000_single",
        mount="right",
        tip_racks=[tiprack]
    )

    # -----------------------------
    # Pick Up Detector Once
    # -----------------------------

    pipette.pick_up_tip(tiprack["A1"])

    # -----------------------------
    # Select Wells
    # -----------------------------

    well_names = ["A1", "A3", "A5"]
    selected_wells = [plate[name] for name in well_names]

    hover_height_mm = 0
    measurement_time_sec = 10

    # -----------------------------
    # Scan Selected Wells
    # -----------------------------

    for well in selected_wells:

        protocol.comment(f"Scanning {well.display_name}")

        pipette.move_to(well.top(z=hover_height_mm))
        protocol.delay(seconds=measurement_time_sec)

    # -----------------------------
    # Return Detector to A1
    # -----------------------------

    protocol.comment("Returning detector to original position (A1)")
    pipette.drop_tip(tiprack["A1"])

    protocol.comment("Protocol complete.")
