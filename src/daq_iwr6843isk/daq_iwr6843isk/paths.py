"""Task output paths anchored in the DAQ workspace, never the home directory."""

from pathlib import Path


def workspace_root():
    for parent in Path(__file__).resolve().parents:
        if ((parent / 'src/daq_bringup/package.xml').is_file()
                and (parent / 'src/daq_iwr6843isk/package.xml').is_file()):
            return parent
    raise RuntimeError('Cannot locate daq_ws sources; pass an explicit output_root')


def default_output_root(kind='radar_trials'):
    return str(workspace_root() / 'outputs' / kind)
