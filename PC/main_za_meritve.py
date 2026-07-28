# V funkciji process_uart_messages:
elif msg.startswith("Razdalja:"):
    if measurement_state in ("meas3", "wait3", "meas4", "wait4"):
        try:
            # Izlušči številski del: odstrani "Razdalja:" in "mm"
            dist_part = msg.split(":")[1].strip()  # npr. "123.45 mm"
            # Odstrani "mm" in morebitne presledke
            dist_str = dist_part.replace("mm", "").strip()
            dist_mm = float(dist_str)
            measurement_distances.append((time.time(), dist_mm))
            print(f"[MERITEV] Zabeležena razdalja: {dist_mm} mm")
        except Exception as e:
            print(f"[MERITEV] Napaka pri razčlenjevanju razdalje: {e} (sporočilo: {msg})")
