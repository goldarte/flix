#!/usr/bin/env python3

"""Show FFT plot for a log entry (time-aware).

Usage:
	fft_new.py [--linear] [--avg=<mode> | --raw] <args>...

Options:
	-l --linear		Show FFT amplitude in linear units (default: dB).
	--avg=<mode>	Averaging mode: welch or raw [default: welch].
	--raw			Shortcut for --avg=raw (single non-averaged FFT).
"""

import csv
import os

import docopt
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.ticker import MultipleLocator
import numpy as np


matplotlib.rcParams['toolbar'] = 'None'


args = docopt.docopt(__doc__)
raw_args = args['<args>']

linear_mode = args['--linear']
avg_mode = (args['--avg'] or "welch").lower()
if args['--raw']:
	avg_mode = 'raw'

if avg_mode not in ('welch', 'raw'):
	print('Invalid --avg mode: ' + str(args['--avg']))
	print('Allowed values: welch, raw')
	quit()

if len(raw_args) < 1:
	print('Usage: fft_new.py [--linear] <csv_file> [<log_entry>]')
	print('   or: fft_new.py [--linear] <csv_file1> [<csv_file2> ...] <log_entry>')
	quit()

if len(raw_args) == 1:
	log_entry = 'motors[0]'
	log_files = raw_args
else:
	log_entry = raw_args[-1]
	log_files = raw_args[:-1]


def calc_fft_for_file(log_file, entry, use_linear=False, averaging='welch'):
	csv_reader = csv.reader(open(log_file, 'r'), delimiter=',')
	header = next(csv_reader, None)

	if not header:
		print(os.path.basename(log_file) + ': CSV is empty')
		return None

	if 't' not in header:
		print(os.path.basename(log_file) + ': Missing required column: t')
		return None

	if entry not in header:
		print(os.path.basename(log_file) + ': Missing requested column: ' + entry)
		return None

	time_index = header.index('t')
	log_entry_index = header.index(entry)

	points = []
	skipped_rows = 0

	for row in csv_reader:
		if len(row) <= max(time_index, log_entry_index):
			skipped_rows += 1
			continue

		t_raw = row[time_index].strip()
		v_raw = row[log_entry_index].strip()

		if not t_raw or not v_raw:
			skipped_rows += 1
			continue

		try:
			t = float(t_raw)
			v = float(v_raw)
		except ValueError:
			skipped_rows += 1
			continue

		if not np.isfinite(t) or not np.isfinite(v):
			skipped_rows += 1
			continue

		points.append((t, v))

	if len(points) < 2:
		print(os.path.basename(log_file) + ': Not enough valid samples to build FFT')
		return None

	points.sort(key=lambda item: item[0])

	# Keep one value per timestamp (average duplicates).
	times = []
	values = []
	for t, v in points:
		if times and t == times[-1]:
			values[-1] = 0.5 * (values[-1] + v)
		else:
			times.append(t)
			values.append(v)

	if len(times) < 2:
		print(os.path.basename(log_file) + ': Not enough unique timestamps to build FFT')
		return None

	duration = times[-1] - times[0]
	if duration <= 0:
		print(os.path.basename(log_file) + ': Invalid time range in column t')
		return None

	# FFT requires uniform sampling. Interpolate measured samples onto uniform time grid.
	N = len(times)
	uniform_times = np.linspace(times[0], times[-1], N)
	uniform_values = np.interp(uniform_times, times, values)

	dt = float(np.mean(np.diff(uniform_times)))
	fs = 1.0 / dt

	# Welch: overlapping Hann-windowed segments averaged in power domain.
	# Raw: single full-length FFT without averaging.
	if averaging == 'raw':
		nperseg = N
		noverlap = 0
		use_hann = False
	else:
		if N < 8:
			nperseg = N
			noverlap = 0
		else:
			nperseg = min(1024, N)
			if nperseg >= 256:
				nperseg = 256
			noverlap = nperseg // 2
		use_hann = True

	step = max(1, nperseg - noverlap)
	segment_count = 0
	avg_power = None

	for start in range(0, N - nperseg + 1, step):
		segment = uniform_values[start:start + nperseg]
		segment = segment - np.mean(segment)

		if use_hann and nperseg >= 3:
			window = np.hanning(nperseg)
		else:
			window = np.ones(nperseg)

		coherent_gain = float(np.mean(window))
		if coherent_gain <= 0:
			coherent_gain = 1.0

		fft_vals = np.fft.rfft(segment * window)
		amp = np.abs(fft_vals) / (nperseg * coherent_gain)
		if len(amp) > 2:
			amp[1:-1] *= 2.0

		power = amp * amp
		if avg_power is None:
			avg_power = power
		else:
			avg_power += power
		segment_count += 1

	if segment_count == 0 or avg_power is None:
		print(os.path.basename(log_file) + ': Not enough data for FFT segments')
		return None

	avg_power /= float(segment_count)
	amplitude = np.sqrt(avg_power)
	freqs = np.fft.rfftfreq(nperseg, d=dt)
	amplitude_db = 20.0 * np.log10(np.maximum(amplitude, 1e-12))

	print(os.path.basename(log_file) + ' Duration: ', duration)
	print(os.path.basename(log_file) + ' Mean sample rate: ', fs)
	print(os.path.basename(log_file) + ' Mean dt: ', dt)
	print(os.path.basename(log_file) + ' Valid rows: ', len(points))
	print(os.path.basename(log_file) + ' Skipped rows: ', skipped_rows)
	print(os.path.basename(log_file) + ' Averaging mode: ', averaging)
	print(os.path.basename(log_file) + ' Segments: ', segment_count)
	print(os.path.basename(log_file) + ' Segment length: ', nperseg)

	if use_linear:
		print(os.path.basename(log_file) + ' Frequency_Hz,Amplitude')
	else:
		print(os.path.basename(log_file) + ' Frequency_Hz,Amplitude_dB')
	for i in range(len(freqs)):
		if use_linear:
			print(freqs[i], ',', amplitude[i])
		else:
			print(freqs[i], ',', amplitude_db[i])

	if use_linear:
		return freqs, amplitude

	return freqs, amplitude_db


plotted = 0
plotted_labels = []
for log_file in log_files:
	result = calc_fft_for_file(log_file, log_entry, use_linear=linear_mode, averaging=avg_mode)
	if result is None:
		continue

	freqs, amplitude = result
	label = os.path.basename(log_file)
	plt.plot(freqs, amplitude, label=label)
	plotted_labels.append(label)
	plotted += 1

if plotted == 0:
	print('No valid sources to plot')
	quit()

if linear_mode:
	plt.title('FFT of ' + log_entry + ' (' + avg_mode + ')')
	plt.ylabel('Amplitude')
	plt.ylim(bottom=0, top=0.04)
else:
	plt.title('FFT of ' + log_entry + ' (dB, ' + avg_mode + ')')
	plt.ylabel('Amplitude (dB)')
	plt.ylim(bottom=-90, top=-35)

plt.xlabel('Frequency (Hz)')
plt.xlim(left=0, right=500)

ax = plt.gca()
ax.xaxis.set_major_locator(MultipleLocator(50))
ax.xaxis.set_minor_locator(MultipleLocator(10))
ax.grid(True, which='major', axis='x', color='#cfcfcf', linewidth=0.9)
ax.grid(True, which='minor', axis='x', color='#e8e8e8', linewidth=0.6)
ax.grid(True, which='major', axis='y', color='#efefef', linewidth=0.8)
if plotted > 1:
	ax.legend()

plt.minorticks_on()
if plotted == 1:
	window_title = log_entry + ' - ' + plotted_labels[0]
else:
	window_title = log_entry + ' - ' + str(plotted) + ' file(s)'
fig_manager = plt.get_current_fig_manager()
if fig_manager is not None:
	fig_manager.set_window_title(window_title)


# plt.gcf().set_size_inches(10, 4)

plt.gcf().set_size_inches(11, 4)

# plt.gcf().set_size_inches(5, 5)

plt.subplots_adjust(bottom=0.18)

plt.show()
