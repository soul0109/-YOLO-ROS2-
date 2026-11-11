/**
 * Per-LED display mapping for archived/live station objects.
 * Does not change YOLO/ROI or archived JSON — display only.
 *
 * Contract (device index leds[0..2]):
 * - green → PASS
 * - off / non-green known bad → FAIL
 * - missing / empty / unknown state → UNKNOWN (never PASS)
 */

export function verdictFromLedState(state) {
  if (state == null || state === '' || state === '—') {
    return 'UNKNOWN'
  }
  const s = String(state).toLowerCase()
  if (s === 'green') return 'PASS'
  if (s === 'unknown') return 'UNKNOWN'
  // off, red, yellow, etc. → FAIL (same spirit as non-green contract)
  return 'FAIL'
}

export function reasonForLedState(state, verdict) {
  if (verdict === 'UNKNOWN') {
    return '缺测/未知状态，无法判定'
  }
  if (verdict === 'PASS') {
    return `本灯状态=${state}`
  }
  return `本灯非绿：状态=${state}`
}

/**
 * @returns {{ name: string, state: string, verdict: string, reason: string }[]}
 */
export function ledLabels(objects) {
  const leds = objects?.leds
  if (!leds) return []
  const states = Array.isArray(leds.model_states) ? leds.model_states : []
  return [0, 1, 2].map((i) => {
    const state = states[i]
    const has = Object.prototype.hasOwnProperty.call(states, i) && state != null && state !== ''
    const displayState = has ? String(state) : '—'
    const verdict = has ? verdictFromLedState(state) : 'UNKNOWN'
    return {
      name: `灯 ${i + 1}（设备 leds[${i}]）`,
      state: displayState,
      verdict,
      reason: reasonForLedState(displayState, verdict),
    }
  })
}

export function ledGroupSummary(objects) {
  const leds = objects?.leds
  if (!leds) return null
  return {
    title: '灯组汇总',
    verdict: leds.verdict || 'UNKNOWN',
    detail:
      `组判定=${leds.verdict || '—'}` +
      (leds.reason ? ` · ${leds.reason}` : '') +
      (Array.isArray(leds.model_states)
        ? ` · model_states=[${leds.model_states.join(',')}]`
        : ''),
  }
}

/**
 * Build UI rows: per-LED, then group summary, then panel/leak.
 */
export function objectRows(objects) {
  if (!objects || typeof objects !== 'object') return []
  const rows = []
  for (const led of ledLabels(objects)) {
    rows.push({
      title: led.name,
      verdict: led.verdict,
      detail: led.reason,
    })
  }
  const summary = ledGroupSummary(objects)
  if (summary) {
    rows.push(summary)
  }
  if (objects.panel) {
    rows.push({
      title: '面板 panel',
      verdict: objects.panel.verdict || 'UNKNOWN',
      detail:
        `状态=${objects.panel.state || '—'}` +
        (objects.panel.reason ? ` · ${objects.panel.reason}` : '') +
        (objects.panel.verdict === 'UNKNOWN' ? ' · 无法判定' : ''),
    })
  }
  if (objects.leak) {
    rows.push({
      title: '漏液 leak',
      verdict: objects.leak.verdict || 'UNKNOWN',
      detail:
        `状态=${objects.leak.state || '—'}` +
        (objects.leak.reason ? ` · ${objects.leak.reason}` : '') +
        (objects.leak.verdict === 'UNKNOWN' ? ' · 无法判定' : ''),
    })
  }
  return rows
}
