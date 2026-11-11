#!/usr/bin/env node
/** Minimal check: [green,green,off] → PASS/PASS/FAIL; missing ≠ PASS. */
import { ledLabels, objectRows, verdictFromLedState } from '../src/ledDisplay.js'

function assert(cond, msg) {
  if (!cond) {
    console.error('FAIL:', msg)
    process.exit(1)
  }
}

const objects = {
  leds: {
    verdict: 'FAIL',
    reason: 'led_non_green:green,green,off',
    model_states: ['green', 'green', 'off'],
  },
  panel: { verdict: 'FAIL', state: 'dark', reason: 'panel_dark' },
  leak: { verdict: 'PASS', state: 'none', reason: 'leak_none' },
}

const leds = ledLabels(objects)
assert(leds.length === 3, 'expect 3 leds')
assert(leds[0].verdict === 'PASS' && leds[1].verdict === 'PASS' && leds[2].verdict === 'FAIL',
  `got ${leds.map((l) => l.verdict).join(',')}`)
assert(!leds[0].reason.includes('led_non_green'), 'per-led reason must not copy group reason')
assert(leds.every((l) => l.reason.includes('本灯') || l.verdict === 'UNKNOWN'), 'per-led reason')

const rows = objectRows(objects)
const summary = rows.find((r) => r.title === '灯组汇总')
assert(summary && summary.verdict === 'FAIL' && summary.detail.includes('led_non_green'),
  'group summary keeps archive group reason')
assert(rows.find((r) => r.title.startsWith('面板')).verdict === 'FAIL', 'panel FAIL')
assert(rows.find((r) => r.title.startsWith('漏液')).verdict === 'PASS', 'leak PASS')

assert(verdictFromLedState('') === 'UNKNOWN', 'empty ≠ PASS')
assert(verdictFromLedState(null) === 'UNKNOWN', 'null ≠ PASS')
assert(verdictFromLedState('unknown') === 'UNKNOWN', 'unknown ≠ PASS')
assert(verdictFromLedState('green') === 'PASS', 'green PASS')
assert(verdictFromLedState('off') === 'FAIL', 'off FAIL')

const missing = ledLabels({ leds: { verdict: 'UNKNOWN', model_states: ['green'] } })
assert(missing[1].verdict === 'UNKNOWN' && missing[2].verdict === 'UNKNOWN', 'missing slots UNKNOWN')
assert(missing[0].verdict === 'PASS', 'present green still PASS')

console.log('OK led display mapping')
