<script setup>
import { computed, onMounted, onUnmounted, ref, watch } from 'vue'
import {
  annotatedUrl,
  createMission,
  getHealth,
  getMission,
  getStations,
  listMissions,
  newMissionId,
  screenshotUrl,
} from './api.js'
import { objectRows } from './ledDisplay.js'
import RuntimeConsole from './RuntimeConsole.vue'

const STORAGE_KEY = 'inspection_r7_selected_mission'
const DEFAULT_ROUTE = 'test_room_ABC'

const health = ref(null)
const healthError = ref('')
const apiOnline = ref(false)
const dataStale = ref(false)
const dataLoadedAt = ref('')
const banner = ref(null) // { type, text }

const submitting = ref(false)
const selectedId = ref('')
const selectedStationId = ref('')
const mission = ref(null)
const stations = ref([])
const history = ref([])
const imageMode = ref('annotated') // screenshot | annotated
const imageBroken = ref(false)
const imageErrorKind = ref('') // unreachable | missing | ''

let pollTimer = null
let healthTimer = null

const observationPending = computed(() => {
  const o = mission.value?.observation_status
  return o === 'PENDING' || o === 'UNKNOWN'
})

const busy = computed(() => {
  if (submitting.value) return true
  if (observationPending.value) return true
  const s = mission.value?.status
  return s === 'ACCEPTED' || s === 'RUNNING'
})

const canSubmit = computed(() => apiOnline.value && !busy.value && !healthError.value)

const connectionLabel = computed(() => {
  if (healthError.value || !apiOnline.value) return { cls: 'bad', text: 'API 不可达' }
  if (!health.value) return { cls: 'warn', text: '检查中…' }
  if (!health.value.ros_bridge) return { cls: 'bad', text: 'ROS bridge 未启动' }
  if (!health.value.action_ready) {
    return { cls: 'warn', text: '已连接 · Runtime Action 不可用（可浏览历史）' }
  }
  return { cls: 'ok', text: '已连接 · Runtime 可用' }
})

const activeStation = computed(() => {
  if (!stations.value.length) return null
  const hit = stations.value.find((s) => s.station_id === selectedStationId.value)
  return hit || stations.value[stations.value.length - 1]
})

const imageSrc = computed(() => {
  if (!selectedId.value || !activeStation.value) return ''
  const sid = activeStation.value.station_id
  return imageMode.value === 'screenshot'
    ? screenshotUrl(selectedId.value, sid)
    : annotatedUrl(selectedId.value, sid)
})

const imageHint = computed(() => {
  if (!activeStation.value) return '暂无站点结果，无法取图。'
  if (!imageBroken.value) return ''
  if (!apiOnline.value || imageErrorKind.value === 'unreachable') {
    return '图片暂时无法加载：API/网络不可达（不是断言文件已缺失）。重连后点刷新即可。'
  }
  return '图片未返回（可能未归档或服务端文件不存在）。通过 API 取图，不用服务器本地路径。'
})

const headline = computed(() => {
  const m = mission.value
  if (!m) return ''
  const insp = m.inspection_status || ''
  if (m.status === 'COMPLETED' && insp === 'FAIL') {
    return {
      type: 'warn',
      text: '任务执行完成，巡检汇总异常（规则：任一站 FAIL → 汇总 FAIL；UNKNOWN 不掩盖 FAIL）',
    }
  }
  if (m.status === 'COMPLETED' && insp === 'PASS') {
    return { type: 'ok', text: '任务执行完成，巡检汇总通过' }
  }
  if (m.status === 'COMPLETED' && insp === 'UNKNOWN') {
    return { type: 'info', text: '任务执行完成，巡检汇总无法判定（无 FAIL，存在 UNKNOWN）' }
  }
  if (m.status === 'COMPLETED') {
    return { type: 'ok', text: '任务执行完成' }
  }
  if (m.status === 'FAILED') {
    return { type: 'error', text: `任务失败：${m.error_code || ''} ${m.message || ''}`.trim() }
  }
  if (m.observation_status === 'PENDING') {
    return {
      type: 'warn',
      text: '观察 PENDING：提交响应超时或迟到接受，持续查询中（不自动重发）',
    }
  }
  if (m.observation_status === 'UNKNOWN') {
    return {
      type: 'warn',
      text: '观察 UNKNOWN：结果通信异常或仅有 RUNNING 壳无 live，不以机器人 FAILED 论处',
    }
  }
  if (m.status === 'RUNNING' || m.status === 'ACCEPTED') {
    return {
      type: 'info',
      text: `执行中 · ${m.phase || m.status}` + (m.station_id ? ` · 站 ${m.station_id}` : ''),
    }
  }
  return { type: 'info', text: m.status }
})

const resultRows = computed(() => objectRows(activeStation.value?.objects))

const progressLine = computed(() => {
  if (!stations.value.length) {
    return mission.value?.status === 'RUNNING' || mission.value?.status === 'ACCEPTED'
      ? '三站进度：等待首站归档…'
      : ''
  }
  return '已归档站点：' + stations.value.map((s) => `${s.station_id}:${s.inspection_status || '?'}`).join(' → ')
})

function markLoaded() {
  dataLoadedAt.value = new Date().toLocaleString()
  dataStale.value = false
}

function persistSelection(id) {
  selectedId.value = id
  if (id) {
    localStorage.setItem(STORAGE_KEY, id)
    const url = new URL(window.location.href)
    url.searchParams.set('mission_id', id)
    window.history.replaceState({}, '', url)
  }
}

function setBanner(type, text) {
  banner.value = { type, text }
}

async function refreshHealth() {
  const wasOnline = apiOnline.value
  try {
    health.value = await getHealth()
    healthError.value = ''
    apiOnline.value = true
    if (!wasOnline && dataStale.value) {
      // reconnect: reload without inventing new conclusions
      await refreshHistory()
      if (selectedId.value) {
        await loadMission(selectedId.value, { quiet: true })
        imageBroken.value = false
        imageErrorKind.value = ''
      }
      setBanner('ok', '已重新连接，已刷新历史/当前任务视图')
    }
  } catch (e) {
    health.value = null
    healthError.value = e.message || String(e)
    apiOnline.value = false
    if (mission.value || history.value.length) {
      dataStale.value = true
    }
  }
}

async function refreshHistory() {
  if (!apiOnline.value && healthError.value) {
    // keep existing history list
    return
  }
  try {
    history.value = await listMissions(20)
    markLoaded()
  } catch (e) {
    apiOnline.value = false
    healthError.value = e.message || String(e)
    dataStale.value = true
    setBanner('error', `无法刷新历史（后端连接问题，保留已显示内容）：${e.message}`)
  }
}

async function loadMission(id, { quiet = false } = {}) {
  if (!id) return
  try {
    const m = await getMission(id)
    mission.value = m
    persistSelection(id)
    markLoaded()
    try {
      stations.value = await getStations(id)
      if (stations.value.length) {
        const ids = stations.value.map((s) => s.station_id)
        if (!ids.includes(selectedStationId.value)) {
          selectedStationId.value = ids[ids.length - 1]
        }
      }
      imageBroken.value = false
      imageErrorKind.value = ''
    } catch (e) {
      // Keep previous stations if any; do not wipe conclusion on transient fail
      if (!stations.value?.length) {
        stations.value = []
      }
      if (!quiet) {
        setBanner('warn', `站点结果暂不可刷新：${e.message}`)
      }
    }
  } catch (e) {
    if (e.status === 404) {
      if (!quiet) setBanner('warn', `未找到 mission_id=${id}`)
      // only clear when server explicitly says not found
      mission.value = null
      stations.value = []
    } else {
      apiOnline.value = false
      healthError.value = e.message || String(e)
      dataStale.value = true
      if (!quiet) {
        setBanner('error', `查询失败（连接/服务），保留已加载数据：${e.message}`)
      }
    }
  }
}

function stopPoll() {
  if (pollTimer) {
    clearInterval(pollTimer)
    pollTimer = null
  }
}

function stillTracking(m) {
  if (!m) return false
  if (m.observation_status === 'PENDING' || m.observation_status === 'UNKNOWN') {
    return true
  }
  return ['ACCEPTED', 'RUNNING'].includes(m.status)
}

function startPoll(id) {
  stopPoll()
  pollTimer = setInterval(async () => {
    await loadMission(id, { quiet: true })
    if (!stillTracking(mission.value)) {
      stopPoll()
      await refreshHistory()
    }
  }, 1500)
}

function selectStation(stationId) {
  selectedStationId.value = stationId
  imageBroken.value = false
  imageErrorKind.value = ''
}

async function onStartABC() {
  banner.value = null
  if (!canSubmit.value) {
    setBanner('error', '后端不可用或任务进行中，禁止提交（连接问题 ≠ 任务失败）')
    return
  }
  if (health.value && !health.value.action_ready) {
    setBanner('warn', 'Runtime Action 不可用（503 风险）。浏览历史无需 Runtime；提交任务请先启动 Runtime。')
    return
  }

  const id = newMissionId()
  submitting.value = true
  mission.value = null
  stations.value = []
  selectedStationId.value = ''
  persistSelection(id)

  try {
    const { status, body } = await createMission(id, DEFAULT_ROUTE)
    if (status !== 202 || !body?.accepted) {
      setBanner('error', `未被接受：${body?.message || status}`)
      submitting.value = false
      return
    }
    setBanner('ok', `已接受：${id}（route=${DEFAULT_ROUTE}）`)
    mission.value = {
      mission_id: id,
      route_id: body.route_id || DEFAULT_ROUTE,
      source: 'live',
      status: body.status || 'ACCEPTED',
      phase: 'VALIDATING',
      error_code: 'NONE',
      message: body.message || '',
      inspection_status: '',
      station_id: '',
      observation_status: body.observation_status || 'CLEAR',
    }
    markLoaded()
    startPoll(id)
  } catch (e) {
    if (e.status === 409) {
      setBanner('error', `冲突：${e.body?.message || e.message}`)
    } else if (e.status === 503) {
      setBanner('error', `服务不可用：${e.body?.message || e.message}`)
    } else if (e.status === 504 || e.code === 'submit_timeout') {
      const body = e.body || {}
      setBanner(
        'warn',
        `观察 PENDING：提交超时，不伪造失败，不自动重发。持续查询 mission_id=${id}。`,
      )
      mission.value = {
        mission_id: id,
        route_id: body.route_id || DEFAULT_ROUTE,
        source: 'live',
        status: body.status || 'ACCEPTED',
        phase: '',
        error_code: body.error_code || 'goal_response_timeout',
        message: body.message || 'observation PENDING after submit timeout',
        inspection_status: '',
        observation_status: body.observation_status || 'PENDING',
      }
      startPoll(id)
    } else {
      setBanner('error', `提交失败（连接/服务）：${e.message}`)
      apiOnline.value = false
      dataStale.value = true
    }
  } finally {
    submitting.value = false
  }
}

async function onSelectHistory(id) {
  banner.value = null
  stopPoll()
  if (!apiOnline.value) {
    setBanner('warn', 'API 不可达：保留当前已加载内容，无法切换到其他历史任务')
    return
  }
  await loadMission(id)
  if (stillTracking(mission.value)) {
    startPoll(id)
  }
}

async function onRefreshAll() {
  banner.value = null
  await refreshHealth()
  if (apiOnline.value) {
    await refreshHistory()
    if (selectedId.value) {
      await loadMission(selectedId.value)
      imageBroken.value = false
      imageErrorKind.value = ''
    }
  }
}

function onImageError() {
  imageBroken.value = true
  imageErrorKind.value = apiOnline.value ? 'missing' : 'unreachable'
}

onMounted(async () => {
  await refreshHealth()
  if (apiOnline.value) {
    await refreshHistory()
  }

  const params = new URLSearchParams(window.location.search)
  const fromUrl = params.get('mission_id')
  const fromStore = localStorage.getItem(STORAGE_KEY)
  const initial = fromUrl || fromStore
  if (initial && apiOnline.value) {
    await loadMission(initial, { quiet: true })
    if (mission.value && stillTracking(mission.value)) {
      startPoll(initial)
    }
  }

  healthTimer = setInterval(refreshHealth, 5000)
})

onUnmounted(() => {
  stopPoll()
  if (healthTimer) clearInterval(healthTimer)
})

watch(imageMode, () => {
  imageBroken.value = false
  imageErrorKind.value = ''
})
</script>

<template>
  <header>
    <h1>巡检控制台 · A→B→C</h1>
    <p class="sub">MVP-R7：网页发起三站任务；仅 HTTP API；不直接控制运动或写库。</p>
  </header>

  <section class="card" style="margin-bottom: 1rem">
    <div class="row" style="align-items: center; justify-content: space-between">
      <div>
        <span class="badge" :class="connectionLabel.cls">{{ connectionLabel.text }}</span>
        <span v-if="health" class="mono" style="margin-left: 0.75rem; color: var(--muted)">
          archive={{ health.archive_exists ? 'yes' : 'no' }}
          · console={{ health.console_hub ? 'on' : 'off' }}
        </span>
      </div>
      <button class="secondary" type="button" @click="onRefreshAll">
        刷新连接/历史
      </button>
    </div>
    <p v-if="healthError" class="alert error">
      后端连接问题：{{ healthError }}（不是机器人任务失败；已加载结论不会因此改写）
    </p>
    <p v-if="dataStale && (mission || history.length)" class="alert warn" style="margin-bottom: 0">
      显示为「上次加载的数据」
      <span v-if="dataLoadedAt">（{{ dataLoadedAt }}）</span>
      — 断线期间禁止提交新任务；重连后点刷新即可。
    </p>
  </section>

  <RuntimeConsole :api-online="apiOnline" />

  <div class="row" style="margin-top: 1rem">
    <section class="card grow">
      <h2>当前任务</h2>
      <button type="button" :disabled="!canSubmit" @click="onStartABC">
        {{ busy ? '任务进行中…' : apiOnline ? '发起 A→B→C 巡检' : 'API 断开 · 不可提交' }}
      </button>
      <p class="sub" style="margin-top: 0.5rem">
        route_id={{ DEFAULT_ROUTE }}；仅 HTTP 202 且 accepted 后显示「已接受」。浏览历史无需 Gazebo。
      </p>

      <div v-if="banner" class="alert" :class="banner.type">{{ banner.text }}</div>
      <div v-if="headline" class="alert" :class="headline.type">{{ headline.text }}</div>
      <p v-if="progressLine" class="sub">{{ progressLine }}</p>

      <template v-if="mission">
        <dl class="meta">
          <dt>mission_id</dt>
          <dd class="mono">{{ mission.mission_id }}</dd>
          <dt>route_id</dt>
          <dd class="mono">{{ mission.route_id || '—' }}</dd>
          <dt>来源</dt>
          <dd>
            {{ mission.source }}
            <span v-if="dataStale" class="badge warn">上次加载</span>
          </dd>
          <dt>执行状态</dt>
          <dd>{{ mission.status }}</dd>
          <dt>观察状态</dt>
          <dd>{{ mission.observation_status || 'CLEAR' }}</dd>
          <dt>阶段</dt>
          <dd>{{ mission.phase || '—' }}</dd>
          <dt>error_code</dt>
          <dd>{{ mission.error_code || 'NONE' }}</dd>
          <dt>巡检汇总</dt>
          <dd>
            <strong>{{ mission.inspection_status || '—' }}</strong>
            <span class="sub">（与执行 COMPLETED 独立；FAIL&gt;UNKNOWN&gt;PASS）</span>
          </dd>
          <dt>说明</dt>
          <dd>{{ mission.message || '—' }}</dd>
        </dl>

        <div v-if="stations.length" class="tabs" style="margin-top: 0.75rem">
          <button
            v-for="s in stations"
            :key="s.station_id"
            type="button"
            :class="selectedStationId === s.station_id ? '' : 'secondary'"
            @click="selectStation(s.station_id)"
          >
            站 {{ s.station_id }} · {{ s.inspection_status || '?' }}
          </button>
        </div>

        <div v-if="activeStation" class="sub" style="margin-top: 0.5rem">
          当前查看站 <strong>{{ activeStation.station_id }}</strong>
          · 站内巡检 {{ activeStation.inspection_status || '—' }}
          · stamp {{ activeStation.image_stamp || '—' }}
        </div>

        <div v-if="resultRows.length" class="objects">
          <div
            v-for="(row, idx) in resultRows"
            :key="idx"
            class="obj"
          >
            <strong>
              {{ row.title }}
              <span
                class="badge"
                :class="row.verdict === 'PASS' ? 'ok' : row.verdict === 'FAIL' ? 'bad' : 'warn'"
              >
                {{ row.verdict === 'UNKNOWN' ? '无法判定' : row.verdict }}
              </span>
            </strong>
            <div>{{ row.detail }}</div>
          </div>
        </div>

        <div v-if="activeStation" class="img-wrap">
          <div class="tabs">
            <button
              type="button"
              :class="imageMode === 'annotated' ? '' : 'secondary'"
              @click="imageMode = 'annotated'"
            >
              标注图
            </button>
            <button
              type="button"
              :class="imageMode === 'screenshot' ? '' : 'secondary'"
              @click="imageMode = 'screenshot'"
            >
              原图
            </button>
          </div>
          <img
            v-if="imageSrc && !imageBroken"
            :src="imageSrc"
            :alt="imageMode + '-' + activeStation.station_id"
            @error="onImageError"
          />
          <p v-else class="sub">{{ imageHint }}</p>
        </div>
      </template>
      <p v-else class="sub">尚未选择任务。可从右侧历史打开，或发起 A→B→C。</p>
    </section>

    <aside class="card narrow">
      <h2>最近归档</h2>
      <ul class="history">
        <li
          v-for="h in history"
          :key="h.mission_id"
          :class="{ active: h.mission_id === selectedId }"
          @click="onSelectHistory(h.mission_id)"
        >
          <div class="id">{{ h.mission_id }}</div>
          <div class="line">{{ h.status }} · {{ h.ended_at || h.started_at }}</div>
        </li>
      </ul>
      <p v-if="!history.length" class="sub">暂无归档记录。</p>
    </aside>
  </div>
</template>
