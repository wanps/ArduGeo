#!/usr/bin/env python3

"""AP_FLAKE8_CLEAN.

Validate the ArduGeo flat-to-module parameter-storage upgrade with two
prebuilt Copter SITL binaries. The old binary writes the former flat layout;
the new binary converts it, accepts new values, and preserves those values
across another wipe=False restart.
"""

import argparse
import hashlib
import math
import os
import shutil
import tempfile

import test_param_upgrade as tpu


# Distinct in-range floating-point values make cross-wiring visible. The three
# Boolean parameters necessarily share 0/1 values. Together these cover every
# moved flat parameter plus two controls whose storage identity did not move.
LEGACY_VALUES = {
    'GEO_POS_KX_XY': 0.01,
    'GEO_POS_KX_Z': 0.02,
    'GEO_POS_KI_XY': 0.03,
    'GEO_POS_KI_Z': 0.04,
    'GEO_POS_KV_XY': 0.05,
    'GEO_POS_KV_Z': 0.06,
    'GEO_ATT_KR_X': 0.07,
    'GEO_ATT_KR_Y': 0.08,
    'GEO_ATT_KR_Z': 0.09,
    'GEO_ATT_KO_X': 0.10,
    'GEO_ATT_KO_Y': 0.11,
    'GEO_ATT_KO_Z': 0.12,
    'GEO_HOV_THR': 0.13,
    'GEO_MOM_NORM_X': 0.14,
    'GEO_MOM_NORM_Y': 0.15,
    'GEO_MOM_NORM_Z': 0.16,
    'GEO_OUT_EN': 0,
    'GEO_POS_FLTE': 0.18,
    'GEO_VEL_FLTE': 0.19,
    'GEO_OMG_FLTE': 0.20,
    'GEO_POS_IMAX_XY': 0.21,
    'GEO_POS_IMAX_Z': 0.22,
    'GEO_ATT_KI_Z': 0.23,
    'GEO_ATT_IMAX_Z': 0.24,
    'GEO_ATT_INT_C': 0.25,
    'GEO_ATT_KI_X': 0.26,
    'GEO_ATT_KI_Y': 0.27,
    'GEO_ATT_IMAX_X': 0.28,
    'GEO_ATT_IMAX_Y': 0.29,
    'GEO_SHAPE_EN': 0,
    'GEO_SHAPE_VXY': 0.31,
    'GEO_SHAPE_AXY': 0.32,
    'GEO_SHAPE_VUP': 0.33,
    'GEO_SHAPE_VDN': 0.34,
    'GEO_SHAPE_AZ': 0.35,
    'GEO_SHAPE_YRAT': 0.36,
    'GEO_SHAPE_YACC': 0.37,
    'GEO_SHAPE_YAW': 0,
    'GEO_ATT_J_X': 0.39,
    'GEO_ATT_J_Y': 0.40,
    'GEO_ATT_J_Z': 0.41,
    'GEO_POS_INT_C': 0.42,
    'GEO_OMG_C_FLT': 0.43,
    'GEO_DOMG_C_FLT': 0.44,
    'GEO_LREF_VXY': 1.23,
}

MOVED_PARAMETER_GROUPS = {
    'position': {
        'GEO_POS_KX_XY',
        'GEO_POS_KX_Z',
        'GEO_POS_KI_XY',
        'GEO_POS_KI_Z',
        'GEO_POS_KV_XY',
        'GEO_POS_KV_Z',
        'GEO_POS_FLTE',
        'GEO_VEL_FLTE',
        'GEO_POS_IMAX_XY',
        'GEO_POS_IMAX_Z',
        'GEO_POS_INT_C',
        'GEO_OMG_C_FLT',
        'GEO_DOMG_C_FLT',
    },
    'attitude': {
        'GEO_ATT_KR_X',
        'GEO_ATT_KR_Y',
        'GEO_ATT_KR_Z',
        'GEO_ATT_KO_X',
        'GEO_ATT_KO_Y',
        'GEO_ATT_KO_Z',
        'GEO_OMG_FLTE',
        'GEO_ATT_KI_X',
        'GEO_ATT_KI_Y',
        'GEO_ATT_KI_Z',
        'GEO_ATT_IMAX_X',
        'GEO_ATT_IMAX_Y',
        'GEO_ATT_IMAX_Z',
        'GEO_ATT_INT_C',
        'GEO_ATT_J_X',
        'GEO_ATT_J_Y',
        'GEO_ATT_J_Z',
    },
    'output_mapper': {
        'GEO_HOV_THR',
        'GEO_MOM_NORM_X',
        'GEO_MOM_NORM_Y',
        'GEO_MOM_NORM_Z',
    },
    'setpoint_shaper': {
        'GEO_SHAPE_EN',
        'GEO_SHAPE_VXY',
        'GEO_SHAPE_AXY',
        'GEO_SHAPE_VUP',
        'GEO_SHAPE_VDN',
        'GEO_SHAPE_AZ',
    },
    'yaw_shaper': {
        'GEO_SHAPE_YRAT',
        'GEO_SHAPE_YACC',
        'GEO_SHAPE_YAW',
    },
}

INT8_PARAMETERS = {
    'GEO_OUT_EN',
    'GEO_SHAPE_EN',
    'GEO_SHAPE_YAW',
}

CONTROL_CANARIES = {
    'GEO_OUT_EN',
    'GEO_LREF_VXY',
}

CURRENT_VALUES = {
    name: (1 if name in INT8_PARAMETERS else round(value + 0.5, 2))
    for name, value in LEGACY_VALUES.items()
}


def binary_path(value):
    """Validate and normalize a Copter SITL binary path."""
    path = os.path.realpath(value)
    if not os.path.isfile(path):
        raise argparse.ArgumentTypeError('binary does not exist: %s' % path)
    if not os.access(path, os.X_OK):
        raise argparse.ArgumentTypeError('binary is not executable: %s' % path)
    if 'copter' not in path.lower():
        raise argparse.ArgumentTypeError(
            "Copter binary path must contain 'copter': %s" % path)
    return path


def sha256(path):
    """Return the SHA-256 digest of a binary."""
    digest = hashlib.sha256()
    with open(path, 'rb') as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def prepare_work_dir(requested_path):
    """Create or validate an empty directory for the shared SITL EEPROM."""
    if requested_path is None:
        return tempfile.mkdtemp(prefix='ardugeo-param-module-upgrade-')

    path = os.path.realpath(requested_path)
    if os.path.exists(path):
        if not os.path.isdir(path):
            raise ValueError(
                'work path exists and is not a directory: %s' % path)
        if os.listdir(path):
            raise ValueError('work directory must be empty: %s' % path)
    else:
        os.makedirs(path)
    return path


def close_suite(suite):
    """Close any MAVLink connection and SITL process owned by a suite."""
    errors = []
    if getattr(suite, 'sitl', None) is not None:
        try:
            suite.stop_SITL()
        except Exception as error:
            errors.append('SITL: %s' % error)
        finally:
            suite.sitl = None
    mav = getattr(suite, 'mav', None)
    if mav is not None:
        try:
            mav.close()
        except Exception as error:
            errors.append('MAVLink: %s' % error)
        finally:
            suite.mav = None
    if errors:
        raise RuntimeError('; '.join(errors))


def run_suite(suite):
    """Run one phase and preserve the original error if cleanup also fails."""
    try:
        suite.run()
    except BaseException:
        try:
            close_suite(suite)
        except Exception as cleanup_error:
            print('ARDUGEO_PARAM_CLEANUP_ERROR=%s' % cleanup_error)
        raise
    else:
        close_suite(suite)


def assert_values(suite, expected, stage, epsilon):
    """Read and compare a complete stage-specific parameter set."""
    actual = suite.get_parameters(sorted(expected))
    failures = []
    for name, wanted in expected.items():
        got = actual[name]
        if not math.isfinite(got) or abs(got - wanted) > epsilon:
            failures.append('%s wanted=%s got=%s' % (name, wanted, got))
    if failures:
        raise ValueError(
            '%s failed:\n%s' % (stage, '\n'.join(failures)))
    print(
        'ARDUGEO_PARAM_STAGE_PASS stage=%s count=%u' %
        (stage, len(expected)))


def snapshot_eeprom(work_dir, name):
    """Copy the current EEPROM to a named retained artifact."""
    source = os.path.join(work_dir, 'eeprom.bin')
    if not os.path.isfile(source):
        raise ValueError('SITL did not create %s' % source)
    destination = os.path.join(work_dir, name)
    shutil.copy2(source, destination)
    return destination


class SetValues(tpu.TestParamUpgradeTestSuite):
    """Write and verify one value set with a selected wipe policy."""

    def __init__(self, binary, values, wipe, stage, epsilon):
        """Construct a parameter-writing SITL phase."""
        super(SetValues, self).__init__(binary)
        self.values = values
        self.wipe = wipe
        self.stage = stage
        self.epsilon = epsilon

    def run(self):
        """Start SITL, write every value, and wait for EEPROM persistence."""
        self.start_SITL(
            binary=self.binary,
            model=self.model(),
            sitl_home='1,1,1,1',
            wipe=self.wipe,
        )
        self.get_mavlink_connection_going()
        self.set_parameters(self.values)
        assert_values(self, self.values, self.stage, self.epsilon)
        self.delay_sim_time(2, reason='EEPROM write to complete')


class CheckValues(tpu.TestParamUpgradeTestSuite):
    """Verify one value set after starting SITL without wiping EEPROM."""

    def __init__(self, binary, values, stage, epsilon):
        """Construct a parameter-reading SITL phase."""
        super(CheckValues, self).__init__(binary)
        self.values = values
        self.stage = stage
        self.epsilon = epsilon

    def run(self):
        """Start SITL without wiping and verify every expected value."""
        self.start_SITL(
            binary=self.binary,
            model=self.model(),
            sitl_home='1,1,1,1',
            wipe=False,
        )
        self.get_mavlink_connection_going()
        assert_values(self, self.values, self.stage, self.epsilon)
        self.delay_sim_time(2, reason='EEPROM write to complete')


def main():
    """Parse CLI arguments and execute the four migration phases."""
    parser = argparse.ArgumentParser(
        description=(
            'Validate ArduGeo flat-to-module Copter parameter migration '
            'and idempotence.'))
    parser.add_argument(
        '--old-binary',
        required=True,
        type=binary_path,
        help='pre-module Copter SITL binary that writes the flat GEO_ layout',
    )
    parser.add_argument(
        '--new-binary',
        required=True,
        type=binary_path,
        help='module-split Copter SITL binary under test',
    )
    parser.add_argument(
        '--work-dir',
        help=(
            'new or empty directory for EEPROM snapshots; '
            'default creates a /tmp directory'),
    )
    parser.add_argument(
        '--epsilon',
        type=float,
        default=1.0e-4,
        help='absolute parameter comparison tolerance (default: 1e-4)',
    )
    args = parser.parse_args()

    old_sha256 = sha256(args.old_binary)
    new_sha256 = sha256(args.new_binary)
    if (os.path.samefile(args.old_binary, args.new_binary) or
            old_sha256 == new_sha256):
        raise ValueError('old and new binaries must be different files')
    if (not math.isfinite(args.epsilon) or
            args.epsilon <= 0 or args.epsilon > 1.0e-3):
        raise ValueError('epsilon must be finite and in (0, 1e-3]')

    grouped_count = sum(
        len(names) for names in MOVED_PARAMETER_GROUPS.values())
    moved_names = set().union(*MOVED_PARAMETER_GROUPS.values())
    expected_names = moved_names | CONTROL_CANARIES
    if grouped_count != 43 or len(moved_names) != 43:
        raise ValueError(
            'moved-parameter groups contain duplicates or wrong count: '
            'entries=%u unique=%u' % (grouped_count, len(moved_names)))
    if set(LEGACY_VALUES) != expected_names:
        missing = sorted(expected_names - set(LEGACY_VALUES))
        unexpected = sorted(set(LEGACY_VALUES) - expected_names)
        raise ValueError(
            'unexpected parameter coverage: missing=%s unexpected=%s' %
            (missing, unexpected))
    unchanged_values = [
        name for name in LEGACY_VALUES
        if LEGACY_VALUES[name] == CURRENT_VALUES[name]
    ]
    if unchanged_values:
        raise ValueError(
            'current test values did not change: %s' % unchanged_values)

    work_dir = prepare_work_dir(args.work_dir)
    print('ARDUGEO_PARAM_WORK_DIR=%s' % work_dir)
    print(
        'ARDUGEO_PARAM_OLD_BINARY=%s sha256=%s' %
        (args.old_binary, old_sha256))
    print(
        'ARDUGEO_PARAM_NEW_BINARY=%s sha256=%s' %
        (args.new_binary, new_sha256))
    print(
        'ARDUGEO_PARAM_SCHEMA_REQUIREMENT='
        'old=pre-module-flat,new=module-split')
    print(
        'ARDUGEO_PARAM_COVERAGE moved=%u canaries=%u total=%u' %
        (len(moved_names), len(CONTROL_CANARIES), len(LEGACY_VALUES)))

    os.chdir(work_dir)

    # Do not use TestParamUpgradeTestSuiteSetParameters here: its current run()
    # reads a module-global param_changes instead of self.param_changes. These
    # local suites keep the generic SITL setup but make tested values explicit.
    run_suite(SetValues(
        args.old_binary,
        LEGACY_VALUES,
        wipe=True,
        stage='old-flat-write',
        epsilon=args.epsilon,
    ))
    snapshot_eeprom(work_dir, 'eeprom-old-flat.bin')

    run_suite(CheckValues(
        args.new_binary,
        LEGACY_VALUES,
        stage='new-converted',
        epsilon=args.epsilon,
    ))
    snapshot_eeprom(work_dir, 'eeprom-after-conversion.bin')

    run_suite(SetValues(
        args.new_binary,
        CURRENT_VALUES,
        wipe=False,
        stage='new-values-write',
        epsilon=args.epsilon,
    ))
    snapshot_eeprom(work_dir, 'eeprom-after-current-values.bin')

    run_suite(CheckValues(
        args.new_binary,
        CURRENT_VALUES,
        stage='new-values-restart',
        epsilon=args.epsilon,
    ))
    snapshot_eeprom(work_dir, 'eeprom-after-restart.bin')

    print(
        'ARDUGEO_PARAM_UPGRADE_PASS moved=%u canaries=%u total=%u' %
        (len(moved_names), len(CONTROL_CANARIES), len(LEGACY_VALUES)))
    print('ARDUGEO_PARAM_ARTIFACTS=%s' % work_dir)


if __name__ == '__main__':
    main()
