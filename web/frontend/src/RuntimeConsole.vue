<script setup>
import { computed, onMounted, onUnmounted, ref, watch } from 'vue'
import { cameraUrl, getConsoleMap, getConsoleTelemetry } from './api.js'

const props = defineProps({
  apiOnline: { type: Boolean, default: false },
})

const telemetry = ref(null)
const mapPayload = ref(null)
const mapError = ref('')
const telError = ref('')
const camBroken = ref(false)
const camTick = ref(0)
const canvasEl = ref(null)

let telTimer = null
let mapTimer = null

const poseFresh = computed(() => telemetry.value?.pose?.freshness || {})
const camFresh = computed(() => telemetry.value?.camera?.freshness || {})
const pathFresh = computed(() => telemetry.value?.path?.freshness || {})

const liveLine = computed(() => {
  const live = telemetry.value?.live_mission
  if (!live) return '无进行中 live 任务'
  return (
    `${live.mission_id} · ${live.status}/${live.phase || '—'} · 站 ${live.station_id || '—'}`
  )
})

const goalLine = computed(() => {
  const g = telemetry.value?.goal
  if (!g) return '无当前目标反馈'
  return `站 ${g.station_id || '—'} · ${g.phase || '—'} · ${g.message || ''}`
})

function freshnessLabel(f) {
  if (!f || f.unknown) return { cls: 'warn', text: '未知/无数据' }
  if (f.stale) return { cls: 'warn', text: `过期 ${f.age_sec}s` }
  return { cls: 'ok', text: `新鲜 ${f.age_sec}s` }
}

async function refreshTelemetry() {
  if (!props.apiOnline) return
  try {
    telemetry.value = await getConsoleTelemetry()
    telError.value = ''
    camTick.value += 1
    draw()
  } catch (e) {
    telError.value = e.message || String(e)
  }
}

async function refreshMap() {
  if (!props.apiOnline) return
  try {
    mapPayload.value = await getConsoleMap()
    mapError.value = ''
    draw()
  } catch (e) {
    mapError.value = e.message || String(e)
  }
}

function occupancyColor(v) {
  if (v < 0) return [40, 48, 58] // unknown
  if (v >= 65) return [28, 28, 32] // occupied
  if (v <= 25) return [232, 236, 240] // free
  return [160, 168, 176]
}

function draw() {
  const canvas = canvasEl.value
  const map = mapPayload.value
  const tel = telemetry.value
  if (!canvas || !map?.available || !map.meta || !map.data) return

  const { width, height } = map.meta
  if (canvas.width !== width || canvas.height !== height) {
    canvas.width = width
    canvas.height = height
  }
  const ctx = canvas.getContext('2d')
  const img = ctx.createImageData(width, height)
  const data = map.data
  for (let j = 0; j < height; j++) {
    for (let i = 0; i < width; i++) {
      const src = j * width + i
      const dstRow = height - 1 - j // flip Y for RViz-like view
      const dst = (dstRow * width + i) * 4
      const [r, g, b] = occupancyColor(data[src])
      img.data[dst] = r
      img.data[dst + 1] = g
      img.data[dst + 2] = b
      img.data[dst + 3] = 255
    }
  }
  ctx.putImageData(img, 0, 0)

  // Trajectory
  const traj = tel?.trajectory?.canvas_points || []
  if (traj.length > 1) {
    ctx.beginPath()
    ctx.strokeStyle = '#1d4f91'
    ctx.lineWidth = 1.5
    traj.forEach((p, idx) => {
      if (idx === 0) ctx.moveTo(p[0], p[1])
      else ctx.lineTo(p[0], p[1])
    })
    ctx.stroke()
  }

  // Plan path
  const pathOk = tel?.path?.displayable
  const pathPts = pathOk ? tel?.path?.canvas_points || [] : []
  if (pathPts.length > 1) {
    ctx.beginPath()
    ctx.strokeStyle = '#0f6e56'
    ctx.lineWidth = 2
    pathPts.forEach((p, idx) => {
      if (idx === 0) ctx.moveTo(p[0], p[1])
      else ctx.lineTo(p[0], p[1])
    })
    ctx.stroke()
  }

  // ABC stations
  const stations = tel?.stations?.canvas || {}
  Object.entries(stations).forEach(([sid, st]) => {
    const c = st?.canvas
    if (!c) return
    ctx.fillStyle = '#b45309'
    ctx.beginPath()
    ctx.arc(c.x, c.y, 4, 0, Math.PI * 2)
    ctx.fill()
    ctx.fillStyle = '#1a2332'
    ctx.font = '12px sans-serif'
    ctx.fillText(sid, c.x + 6, c.y - 6)
  })

  // Robot pose + heading
  const pose = tel?.pose?.canvas
  const poseStale = tel?.pose?.freshness?.stale || tel?.pose?.freshness?.unknown
  if (pose) {
    ctx.save()
    ctx.translate(pose.x, pose.y)
    ctx.rotate(pose.yaw_canvas)
    ctx.fillStyle = poseStale ? '#9ca3af' : '#b42318'
    ctx.beginPath()
    ctx.moveTo(10, 0)
    ctx.lineTo(-6, 5)
    ctx.lineTo(-6, -5)
    ctx.closePath()
    ctx.fill()
    ctx.restore()
  }
}

const camSrc = computed(() => {
  if (!props.apiOnline) return ''
  // cache-bust; still not evidence
  return `${cameraUrl()}?t=${camTick.value}`
})

function onCamError() {
  camBroken.value = true
}

watch(
  () => props.apiOnline,
  (online) => {
    if (online) {
      refreshMap()
      refreshTelemetry()
    }
  },
)

onMounted(() => {
  if (props.apiOnline) {
    refreshMap()
    refreshTelemetry()
  }
  telTimer = setInterval(refreshTelemetry, 1000)
  mapTimer = setInterval(refreshMap, 10000)
})

onUnmounted(() => {
  if (telTimer) clearInterval(telTimer)
  if (mapTimer) clearInterval(mapTimer)
})
</script>

<template>
  <section class="card console-card">
    <h2>运行监视（只读）</h2>
    <p class="sub">
      地图/位姿/路径/轨迹/低帧率原相机。实时画面<strong>不是</strong>正式到站证据；归档原图/标注图才是。
    </p>

    <div class="console-grid">
      <div class="console-map-wrap">
        <canvas ref="canvasEl" class="console-canvas" />
        <p v-if="mapError" class="alert warn">地图：{{ mapError }}</p>
        <p v-else-if="!mapPayload?.available" class="sub">等待 /map OccupancyGrid…</p>
        <p v-else class="sub">
          map {{ mapPayload.meta?.width }}×{{ mapPayload.meta?.height }}
          · res {{ mapPayload.meta?.resolution }}
          ·
          <span class="badge" :class="freshnessLabel(mapPayload.freshness).cls">
            {{ freshnessLabel(mapPayload.freshness).text }}
          </span>
        </p>
      </div>

      <div class="console-side">
        <dl class="meta">
          <dt>位姿</dt>
          <dd>
            <span class="badge" :class="freshnessLabel(poseFresh).cls">
              {{ freshnessLabel(poseFresh).text }}
            </span>
            <span v-if="telemetry?.pose?.map" class="mono">
              x={{ telemetry.pose.map.x.toFixed(2) }}
              y={{ telemetry.pose.map.y.toFixed(2) }}
              yaw={{ telemetry.pose.map.yaw.toFixed(2) }}
            </span>
            <span v-else>无 amcl_pose</span>
          </dd>
          <dt>规划路径</dt>
          <dd>
            <span class="badge" :class="freshnessLabel(pathFresh).cls">
              {{ freshnessLabel(pathFresh).text }}
            </span>
            <span v-if="telemetry?.path && !telemetry.path.displayable" class="badge warn">
              frame 不匹配 · 不显示
            </span>
            <span v-else>
              {{ telemetry?.path?.canvas_points?.length || 0 }} pts
            </span>
          </dd>
          <dt>会话轨迹</dt>
          <dd>{{ telemetry?.trajectory?.count || 0 }} / {{ telemetry?.trajectory?.max || 0 }}</dd>
          <dt>ABC</dt>
          <dd class="mono">{{ (telemetry?.stations?.order || []).join(' → ') || '—' }}</dd>
          <dt>当前目标</dt>
          <dd>{{ goalLine }}</dd>
          <dt>live 任务</dt>
          <dd class="mono">{{ liveLine }}</dd>
        </dl>

        <div class="cam-box">
          <div class="tabs" style="margin-bottom: 0.35rem">
            <span class="badge" :class="freshnessLabel(camFresh).cls">
              相机预览 · {{ freshnessLabel(camFresh).text }}
            </span>
          </div>
          <img
            v-if="camSrc && !camBroken && !(camFresh.unknown)"
            :src="camSrc"
            alt="live-camera-preview-not-evidence"
            class="cam-img"
            @error="onCamError"
          />
          <p v-else class="sub">
            {{ camFresh.unknown || camBroken ? '相机无数据或断流（过期/未知）' : '等待画面…' }}
          </p>
          <p class="sub">预览 ≠ 到站证据；请用下方归档截图。</p>
        </div>
        <p v-if="telError" class="alert warn">遥测：{{ telError }}</p>
      </div>
    </div>
  </section>
</template>
