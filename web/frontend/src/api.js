/**
 * API base:
 * - VITE_API_BASE=http://VM_IP:8000 → direct backend (CORS enabled)
 * - Vite DEV (no base) → `/api/...` (vite.config proxy → :8000, strip /api)
 * - Production build served by FastAPI :8000 → same-origin root paths
 */
const RAW_BASE = (import.meta.env.VITE_API_BASE || '').replace(/\/$/, '')

export function apiUrl(path) {
  const p = path.startsWith('/') ? path : `/${path}`
  if (RAW_BASE) {
    return `${RAW_BASE}${p}`
  }
  if (import.meta.env.DEV) {
    return `/api${p}`
  }
  return p
}

export async function getHealth() {
  const r = await fetch(apiUrl('/health'), { cache: 'no-store' })
  if (!r.ok) {
    throw new Error(`health HTTP ${r.status}`)
  }
  return r.json()
}

export async function listMissions(limit = 20) {
  const r = await fetch(apiUrl(`/missions?limit=${limit}`), { cache: 'no-store' })
  if (!r.ok) {
    const body = await r.json().catch(() => ({}))
    const err = new Error(body.message || body.detail?.message || `list HTTP ${r.status}`)
    err.status = r.status
    err.body = body
    throw err
  }
  return r.json()
}

export async function getMission(missionId) {
  const r = await fetch(apiUrl(`/missions/${encodeURIComponent(missionId)}`), {
    cache: 'no-store',
  })
  if (!r.ok) {
    const body = await r.json().catch(() => ({}))
    const err = new Error(
      body.detail?.message || body.message || `mission HTTP ${r.status}`,
    )
    err.status = r.status
    err.body = body.detail || body
    throw err
  }
  return r.json()
}

export async function getStations(missionId) {
  const r = await fetch(
    apiUrl(`/missions/${encodeURIComponent(missionId)}/stations`),
    { cache: 'no-store' },
  )
  if (!r.ok) {
    const body = await r.json().catch(() => ({}))
    const err = new Error(
      body.detail?.message || body.message || `stations HTTP ${r.status}`,
    )
    err.status = r.status
    err.body = body.detail || body
    throw err
  }
  return r.json()
}

export async function createMission(missionId, routeId = 'test_room_ABC') {
  const controller = new AbortController()
  const timer = setTimeout(() => controller.abort(), 20000)
  try {
    const r = await fetch(apiUrl('/missions'), {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ mission_id: missionId, route_id: routeId }),
      signal: controller.signal,
    })
    const body = await r.json().catch(() => ({}))
    if (!r.ok) {
      const err = new Error(body.message || `create HTTP ${r.status}`)
      err.status = r.status
      err.body = body
      // 504 may include observation_status=PENDING — caller keeps tracking.
      throw err
    }
    return { status: r.status, body }
  } catch (e) {
    if (e.name === 'AbortError') {
      const err = new Error('提交超时：Runtime 接受结果未知')
      err.status = 504
      err.code = 'submit_timeout'
      throw err
    }
    throw e
  } finally {
    clearTimeout(timer)
  }
}

export function screenshotUrl(missionId, stationId = 'A') {
  return apiUrl(
    `/missions/${encodeURIComponent(missionId)}/stations/${encodeURIComponent(stationId)}/screenshot`,
  )
}

export function annotatedUrl(missionId, stationId = 'A') {
  return apiUrl(
    `/missions/${encodeURIComponent(missionId)}/stations/${encodeURIComponent(stationId)}/annotated`,
  )
}

export function newMissionId() {
  const ts = new Date()
  const pad = (n, w = 2) => String(n).padStart(w, '0')
  const stamp =
    `${ts.getFullYear()}${pad(ts.getMonth() + 1)}${pad(ts.getDate())}_` +
    `${pad(ts.getHours())}${pad(ts.getMinutes())}${pad(ts.getSeconds())}`
  const rnd = Math.random().toString(36).slice(2, 6)
  return `mvp_r7_abc_${stamp}_${rnd}`
}

export async function getConsoleTelemetry() {
  const r = await fetch(apiUrl('/console/telemetry'), { cache: 'no-store' })
  if (!r.ok) {
    const body = await r.json().catch(() => ({}))
    const err = new Error(body.message || body.detail?.message || `telemetry HTTP ${r.status}`)
    err.status = r.status
    err.body = body
    throw err
  }
  return r.json()
}

export async function getConsoleMap() {
  const r = await fetch(apiUrl('/console/map'), { cache: 'no-store' })
  if (!r.ok) {
    const body = await r.json().catch(() => ({}))
    const err = new Error(body.message || body.detail?.message || `map HTTP ${r.status}`)
    err.status = r.status
    err.body = body
    throw err
  }
  return r.json()
}

export function cameraUrl() {
  return apiUrl('/console/camera.jpg')
}

