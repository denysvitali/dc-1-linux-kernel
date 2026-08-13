#!/bin/sh
# SPDX-License-Identifier: GPL-2.0-only

set -eu

if [ "$#" -ne 1 ]; then
	echo "usage: $0 <mt8781-daylight-jagar.dtb>" >&2
	exit 2
fi

dtb=$1
fdtget=${FDTGET:-scripts/dtc/fdtget}

fail()
{
	echo "jagar microSD DT check: $*" >&2
	exit 1
}

symbol_path()
{
	"$fdtget" -ts "$dtb" /__symbols__ "$1" 2>/dev/null ||
		fail "missing symbol $1 (build the DTB with DTC_FLAGS=-@)"
}

read_u32()
{
	"$fdtget" -tu "$dtb" "$1" "$2" 2>/dev/null ||
		fail "missing $1:$2"
}

read_string()
{
	"$fdtget" -ts "$dtb" "$1" "$2" 2>/dev/null ||
		fail "missing $1:$2"
}

assert_u32()
{
	actual=$(read_u32 "$1" "$2")
	[ "$actual" = "$3" ] ||
		fail "$1:$2 is '$actual', expected '$3'"
}

assert_string()
{
	actual=$(read_string "$1" "$2")
	[ "$actual" = "$3" ] ||
		fail "$1:$2 is '$actual', expected '$3'"
}

has_property()
{
	"$fdtget" -p "$dtb" "$1" 2>/dev/null | grep -Fxq "$2"
}

require_property()
{
	has_property "$1" "$2" || fail "missing $1:$2"
}

forbid_property()
{
	if has_property "$1" "$2"; then
		fail "forbidden $1:$2"
	fi
}

assert_phandle()
{
	consumer=$(read_u32 "$1" "$2")
	provider=$(read_u32 "$3" phandle)
	[ "$consumer" = "$provider" ] ||
		fail "$1:$2 does not reference $3"
}

mmc=$(symbol_path mmc0)
pins=$(symbol_path mmc0_pins_default)
pio=$(symbol_path pio)
vmmc=$(symbol_path mt6366_vmch_reg)
vsim2=$(symbol_path mt6366_vsim2_reg)
vqmmc=$(symbol_path reg_vqmmc_sd)
regulators=${vmmc%/*}

# The factory VMCH_EINT_HIGH sequence is not ported: its MT6366 register map
# is not available from a primary source. Reject any node that claims it, so a
# guessed PMIC register write cannot reappear silently.
if "$fdtget" -l "$dtb" "$regulators" 2>/dev/null | grep -Fxq vmch-eint-high; then
	fail "vmch-eint-high has no verified register map; do not ship it"
fi

assert_string "$mmc" pinctrl-names "default state_uhs"
assert_u32 "$mmc" pinctrl-0 "$(read_u32 "$pins" phandle)"
assert_u32 "$mmc" pinctrl-1 "$(read_u32 "$pins" phandle)"
assert_u32 "$mmc" bus-width "4"
assert_u32 "$mmc" max-frequency "25000000"
assert_u32 "$mmc" cd-gpios "$(read_u32 "$pio" phandle) 7 1"
assert_phandle "$mmc" vmmc-supply "$vmmc"
assert_phandle "$mmc" vqmmc-supply "$vqmmc"

for prop in no-mmc no-sdio disable-wp; do
	require_property "$mmc" "$prop"
done

for prop in cap-sd-highspeed sd-uhs-sdr12 sd-uhs-sdr25 sd-uhs-sdr50 \
		sd-uhs-sdr104 sd-uhs-ddr50 operating-points-v2 \
		mediatek,dvfsrc; do
	forbid_property "$mmc" "$prop"
done

assert_u32 "$pins/pins-cd" pinmux "$((7 << 8))"
require_property "$pins/pins-cd" bias-pull-up
assert_u32 "$pins/pins-clk" pinmux "$(((55 << 8) | 1))"
assert_u32 "$pins/pins-clk" drive-strength "3"
require_property "$pins/pins-clk" input-enable
cmd_dat_pinmux="$(((56 << 8) | 1)) $(((58 << 8) | 1))"
cmd_dat_pinmux="$cmd_dat_pinmux $(((59 << 8) | 1)) $(((60 << 8) | 1))"
cmd_dat_pinmux="$cmd_dat_pinmux $(((61 << 8) | 1))"
assert_u32 "$pins/pins-cmd-dat" pinmux \
	"$cmd_dat_pinmux"
assert_u32 "$pins/pins-cmd-dat" drive-strength "3"
require_property "$pins/pins-cmd-dat" input-enable

# The factory board tree caps VMCH at 3.0 V; mainline builds the slot's OCR
# mask from these constraints, so a 3.3 V maximum is a real over-voltage risk.
assert_string "$vmmc" regulator-name "vmch"
assert_u32 "$vmmc" regulator-min-microvolt "2900000"
assert_u32 "$vmmc" regulator-max-microvolt "3000000"

assert_string "$vsim2" regulator-name "vsim2"
assert_u32 "$vsim2" regulator-min-microvolt "1800000"
assert_u32 "$vsim2" regulator-max-microvolt "1800000"

assert_u32 "$vqmmc" regulator-min-microvolt "3000000"
assert_u32 "$vqmmc" regulator-max-microvolt "3000000"
assert_u32 "$vqmmc" gpios \
	"$(read_u32 "$pio" phandle) 157 0"
assert_u32 "$vqmmc" gpios-states "0"
assert_u32 "$vqmmc" states "1800000 1 3000000 0"
assert_phandle "$vqmmc" vin-supply "$vsim2"
forbid_property "$vqmmc" enable-active-high

echo "jagar microSD DT check: PASS"
