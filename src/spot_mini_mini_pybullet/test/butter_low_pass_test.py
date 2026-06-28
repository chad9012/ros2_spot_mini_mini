from scipy.signal import butter, filtfilt

def butter_lowpass_filter(data, cutoff, fs, order=2):

    """ Pass two subsequent datapoints in here to be filtered

    """

    nyq = 0.5 * fs  # Nyquist Frequency

    normal_cutoff = cutoff / nyq

    # Get the filter coefficients

    b, a = butter(order, normal_cutoff, btype='low', analog=False)

    y = filtfilt(b, a, data)

    return y

