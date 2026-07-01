const DT = 0.08;
const MAX_SPEED = 4.0;
const FLOW_PHASE_COUNT = 9;
const STABLE_ERROR = 0.04;
const STABLE_SPEED = 0.05;
const STABLE_CONTROL = 0.05;
const STABLE_TAIL_SECONDS = 5;

const I18N = {
  "zh-CN": {
    appTitle: "预测控制与反馈控制模拟器",
    appSubtitle: "NMPC/MPC 基于模型滚动预测优化；LQR/PID 作为当前状态反馈基线。",
    infoButton: "说明", speed: "倍速", pause: "暂停", resume: "继续", reset: "重置",
    metricX: "位置 x", metricV: "速度 v", metricU: "控制 u", metricCost: "代价 cost", metricCandidates: "候选数",
    flowTitle: "实时控制流", waiting: "等待数据", resetParams: "重置参数", resetAll: "重置全部",
    sceneParams: "场景参数", targetPosition: "目标位置", drag: "真实阻力", slowStart: "限速起点", slowSpeed: "限速上限", slowWeight: "限速惩罚",
    predictiveParams: "预测参数", nmpcParams: "NMPC 参数", mpcParams: "MPC 参数", levels: "候选档位", posWeight: "位置权重", terminalWeight: "终端权重", velocityWeight: "速度权重", smoothWeight: "平滑权重",
    nmpcAdvancedParams: "NMPC 高级参数", mpcAdvancedParams: "MPC 高级参数", controlBlocks: "控制分段数", controlEffortWeight: "控制能量权重", terminalVelocityMultiplier: "终端速度倍率",
    terminalPositionBoost: "终端位置增强", terminalVelocityBoost: "终端速度增强", settleRadius: "收敛半径", settleWeight: "收敛速度权重",
    holdError: "保持误差阈值", holdSpeed: "保持速度阈值", captureError: "捕获误差范围", captureSpeed: "捕获速度范围",
    captureKx: "捕获位置增益", captureKv: "捕获速度增益", captureLimit: "捕获控制限幅",
    pidParams: "PID 反馈控制参数", kp: "比例增益 Kp", ki: "积分增益 Ki", kd: "微分增益 Kd", integralLimit: "积分限幅",
    lqrParams: "LQR 状态反馈参数", lqrKx: "位置反馈 Kx", lqrKv: "速度反馈 Kv",
    infoTitle: "控制器说明", infoNmpc: "NMPC：用非线性模型预测未来。", infoMpc: "MPC：用线性模型预测未来。", infoFeedback: "LQR/PID：不预测未来，只根据当前状态反馈。", infoSlow: "橙色区域：速度约束区，预测控制会提前考虑。",
    running: "运行中", paused: "已暂停", stableRecording: "稳定记录中", stable: "已稳定",
    nonlinearModel: "非线性模型", linearModel: "线性模型", modeNmpc: "NMPC 非线性预测", modeMpc: "MPC 线性预测", modeLqr: "LQR 状态反馈", modePid: "PID 反馈控制",
    slowZone: "限速区", target: "目标", error: "误差", mode: "模式",
    chartPosition: "位置", chartVelocity: "速度", chartControl: "控制量", chartTarget: "目标",
    closedLoopLabel: "闭环反馈：对象新状态 x,v 回到下一次控制器输入", stateData: "状态数据", candidateLoop: "候选循环",
    flowFrozen: "稳定尾段结束，实时控制流已停止", closedLoopShort: "闭环反馈", waitFirstSolve: "等待第一次求解", outputThenFeedback: "闭环：输出后反馈下一帧",
    targetHold: "目标保持中", lockedTarget: "x 已锁定目标", targetCapture: "目标捕获中", dampingSettle: "阻尼收敛",
    candidates: "候选", predictedTerminalX: "预测终点 x", slowHits: "限速命中", kxTerm: "Kx 项", kvTerm: "Kv 项",
    readState: "读取状态", stateVars: "x, v, target", generateCandidates: "生成候选", segmentedControl: "分段控制序列", iterateCandidates: "遍历候选",
    calcCost: "计算 cost", costTerms: "目标/速度/限速", saveBest: "保存 best", costLower: "cost 更小?", executeU0: "执行 u0", firstStepOnly: "只执行第一步",
    plantUpdate: "对象更新", stateChanged: "x,v 发生变化", feedbackNext: "反馈下一帧", rereadState: "重新读取状态",
    readError: "读取误差", pTerm: "P 项", iTerm: "I 项", dTerm: "D 项", accumulatedError: "累计误差", combineU: "合成 u", pidSum: "P + I + D",
    clamp: "限幅", uRange: "u 在 -1 到 1", outputControl: "输出控制", applyToPlant: "作用到对象", rereadError: "重新读取误差",
    positionError: "位置误差", kxFeedback: "Kx 反馈", kvFeedback: "Kv 反馈", stateFeedback: "状态反馈"
  },
  en: {
    appTitle: "Predictive and Feedback Control Simulator",
    appSubtitle: "NMPC/MPC use model-based receding-horizon optimization; LQR/PID are current-state feedback baselines.",
    infoButton: "Info", speed: "Speed", pause: "Pause", resume: "Resume", reset: "Reset",
    metricX: "Position x", metricV: "Velocity v", metricU: "Control u", metricCost: "Cost", metricCandidates: "Candidates",
    flowTitle: "Live Control Flow", waiting: "Waiting for data", resetParams: "Reset Params", resetAll: "Reset All",
    sceneParams: "Scene Parameters", targetPosition: "Target Position", drag: "Plant Drag", slowStart: "Speed Zone Start", slowSpeed: "Speed Limit", slowWeight: "Limit Penalty",
    predictiveParams: "Predictive Parameters", nmpcParams: "NMPC Parameters", mpcParams: "MPC Parameters", levels: "Control Levels", posWeight: "Position Weight", terminalWeight: "Terminal Weight", velocityWeight: "Velocity Weight", smoothWeight: "Smoothness Weight",
    nmpcAdvancedParams: "NMPC Advanced Parameters", mpcAdvancedParams: "MPC Advanced Parameters", controlBlocks: "Control Blocks", controlEffortWeight: "Control Effort Weight", terminalVelocityMultiplier: "Terminal Velocity Multiplier",
    terminalPositionBoost: "Terminal Position Boost", terminalVelocityBoost: "Terminal Velocity Boost", settleRadius: "Settling Radius", settleWeight: "Settling Velocity Weight",
    holdError: "Hold Error Threshold", holdSpeed: "Hold Speed Threshold", captureError: "Capture Error Range", captureSpeed: "Capture Speed Range",
    captureKx: "Capture Position Gain", captureKv: "Capture Velocity Gain", captureLimit: "Capture Control Limit",
    pidParams: "PID Feedback Parameters", kp: "Proportional Gain Kp", ki: "Integral Gain Ki", kd: "Derivative Gain Kd", integralLimit: "Integral Limit",
    lqrParams: "LQR State Feedback Parameters", lqrKx: "Position Feedback Kx", lqrKv: "Velocity Feedback Kv",
    infoTitle: "Controller Notes", infoNmpc: "NMPC: predicts the future with a nonlinear model.", infoMpc: "MPC: predicts the future with a linear model.", infoFeedback: "LQR/PID: do not predict ahead; they react to the current state.", infoSlow: "Orange zone: a speed constraint that predictive controllers consider in advance.",
    running: "Running", paused: "Paused", stableRecording: "Stable tail recording", stable: "Stable",
    nonlinearModel: "Nonlinear model", linearModel: "Linear model", modeNmpc: "NMPC Nonlinear Prediction", modeMpc: "MPC Linear Prediction", modeLqr: "LQR State Feedback", modePid: "PID Feedback Control",
    slowZone: "Speed zone", target: "Target", error: "Error", mode: "Mode", chartPosition: "Position", chartVelocity: "Velocity", chartControl: "Control", chartTarget: "Target",
    closedLoopLabel: "Closed loop: updated plant state x,v returns to the next controller input", stateData: "State data", candidateLoop: "Candidate loop",
    flowFrozen: "Stable tail complete; live control flow stopped", closedLoopShort: "Closed-loop feedback", waitFirstSolve: "Waiting for first solve", outputThenFeedback: "Closed loop: output then feedback next frame",
    targetHold: "Holding target", lockedTarget: "x locked to target", targetCapture: "Capturing target", dampingSettle: "Damped settling",
    candidates: "Candidates", predictedTerminalX: "terminal x", slowHits: "speed hits", kxTerm: "Kx term", kvTerm: "Kv term",
    readState: "Read State", stateVars: "x, v, target", generateCandidates: "Generate", segmentedControl: "segmented controls", iterateCandidates: "Iterate",
    calcCost: "Cost", costTerms: "target/velocity/limit", saveBest: "Save Best", costLower: "lower cost?", executeU0: "Execute u0", firstStepOnly: "first step only",
    plantUpdate: "Plant Update", stateChanged: "x,v changed", feedbackNext: "Feedback Next", rereadState: "read state again",
    readError: "Read Error", pTerm: "P term", iTerm: "I term", dTerm: "D term", accumulatedError: "accumulated error", combineU: "Combine u", pidSum: "P + I + D",
    clamp: "Clamp", uRange: "u in -1 to 1", outputControl: "Output", applyToPlant: "apply to plant", rereadError: "read error again",
    positionError: "Position Error", kxFeedback: "Kx Feedback", kvFeedback: "Kv Feedback", stateFeedback: "state feedback"
  },
  "zh-TW": {},
  ja: {},
  ru: {}
};

Object.assign(I18N["zh-TW"], I18N["zh-CN"], {
  appTitle: "預測控制與回授控制模擬器", appSubtitle: "NMPC/MPC 基於模型滾動預測最佳化；LQR/PID 作為目前狀態回授基線。",
  infoButton: "說明", pause: "暫停", resume: "繼續", metricCost: "代價 cost", flowTitle: "即時控制流", waiting: "等待資料",
  resetAll: "全部重置", sceneParams: "場景參數", targetPosition: "目標位置", slowStart: "限速起點", slowSpeed: "限速上限", slowWeight: "限速懲罰",
  predictiveParams: "預測參數", nmpcParams: "NMPC 參數", mpcParams: "MPC 參數", nmpcAdvancedParams: "NMPC 進階參數", mpcAdvancedParams: "MPC 進階參數", levels: "候選檔位", posWeight: "位置權重", terminalWeight: "終端權重", velocityWeight: "速度權重", smoothWeight: "平滑權重",
  pidParams: "PID 回授控制參數", ki: "積分增益 Ki", integralLimit: "積分限幅", lqrParams: "LQR 狀態回授參數", lqrKx: "位置回授 Kx", lqrKv: "速度回授 Kv",
  infoTitle: "控制器說明", infoNmpc: "NMPC：用非線性模型預測未來。", infoMpc: "MPC：用線性模型預測未來。", infoFeedback: "LQR/PID：不預測未來，只根據目前狀態回授。", infoSlow: "橙色區域：速度約束區，預測控制會提前考慮。",
  running: "運行中", paused: "已暫停", stableRecording: "穩定尾段記錄中", stable: "已穩定", nonlinearModel: "非線性模型", linearModel: "線性模型",
  modeNmpc: "NMPC 非線性預測", modeMpc: "MPC 線性預測", modeLqr: "LQR 狀態回授", modePid: "PID 回授控制", target: "目標", error: "誤差",
  chartControl: "控制量", closedLoopLabel: "閉環回授：物件新狀態 x,v 回到下一次控制器輸入", stateData: "狀態資料", candidateLoop: "候選迴圈",
  flowFrozen: "穩定尾段結束，即時控制流已停止", closedLoopShort: "閉環回授", targetHold: "目標保持中", lockedTarget: "x 已鎖定目標", targetCapture: "目標捕獲中", dampingSettle: "阻尼收斂",
  predictedTerminalX: "預測終點 x", slowHits: "限速命中", readState: "讀取狀態", generateCandidates: "生成候選", iterateCandidates: "遍歷候選", calcCost: "計算 cost",
  saveBest: "保存 best", executeU0: "執行 u0", plantUpdate: "物件更新", feedbackNext: "回授下一幀", rereadState: "重新讀取狀態", readError: "讀取誤差",
  accumulatedError: "累計誤差", combineU: "合成 u", clamp: "限幅", outputControl: "輸出控制", positionError: "位置誤差", stateFeedback: "狀態回授"
});

Object.assign(I18N.ja, I18N.en, {
  appTitle: "予測制御とフィードバック制御シミュレータ", appSubtitle: "NMPC/MPC はモデルに基づく移動ホライズン最適化、LQR/PID は現在状態フィードバックの基準です。",
  infoButton: "説明", speed: "速度", pause: "一時停止", resume: "再開", reset: "リセット", metricX: "位置 x", metricV: "速度 v", metricU: "制御 u", metricCost: "コスト", metricCandidates: "候補数",
  flowTitle: "リアルタイム制御フロー", waiting: "データ待ち", resetParams: "パラメータ初期化", resetAll: "すべて初期化", sceneParams: "シーン設定", targetPosition: "目標位置", drag: "実プラント抵抗",
  slowStart: "速度制約開始", slowSpeed: "速度上限", slowWeight: "制約ペナルティ", predictiveParams: "予測パラメータ", nmpcParams: "NMPC パラメータ", mpcParams: "MPC パラメータ", nmpcAdvancedParams: "NMPC 詳細パラメータ", mpcAdvancedParams: "MPC 詳細パラメータ", levels: "制御候補段数", posWeight: "位置重み",
  terminalWeight: "終端重み", velocityWeight: "速度重み", smoothWeight: "平滑化重み", pidParams: "PID フィードバックパラメータ", kp: "比例ゲイン Kp", ki: "積分ゲイン Ki", kd: "微分ゲイン Kd",
  integralLimit: "積分上限", lqrParams: "LQR 状態フィードバックパラメータ", lqrKx: "位置フィードバック Kx", lqrKv: "速度フィードバック Kv",
  infoTitle: "コントローラ説明", infoNmpc: "NMPC：非線形モデルで未来を予測します。", infoMpc: "MPC：線形モデルで未来を予測します。", infoFeedback: "LQR/PID：未来は予測せず、現在状態に基づいてフィードバックします。", infoSlow: "オレンジ領域：速度制約区間。予測制御は事前に考慮します。",
  running: "実行中", paused: "一時停止中", stableRecording: "安定後記録中", stable: "安定", nonlinearModel: "非線形モデル", linearModel: "線形モデル",
  modeNmpc: "NMPC 非線形予測", modeMpc: "MPC 線形予測", modeLqr: "LQR 状態フィードバック", modePid: "PID フィードバック制御", slowZone: "速度制約区間", target: "目標", error: "誤差", mode: "モード",
  chartPosition: "位置", chartVelocity: "速度", chartControl: "制御量", chartTarget: "目標", closedLoopLabel: "閉ループ：更新後の状態 x,v が次の制御入力へ戻る", stateData: "状態データ", candidateLoop: "候補ループ",
  flowFrozen: "安定後記録が終了し、リアルタイム制御フローは停止しました", closedLoopShort: "閉ループフィードバック", waitFirstSolve: "初回計算待ち", outputThenFeedback: "閉ループ：出力後に次フレームへフィードバック",
  targetHold: "目標保持中", lockedTarget: "x は目標に固定", targetCapture: "目標捕獲中", dampingSettle: "減衰収束", candidates: "候補", predictedTerminalX: "予測終端 x", slowHits: "速度制約命中",
  readState: "状態読取", generateCandidates: "候補生成", iterateCandidates: "候補走査", calcCost: "cost 計算", saveBest: "best 保存", executeU0: "u0 実行", plantUpdate: "プラント更新", feedbackNext: "次へフィードバック",
  readError: "誤差読取", accumulatedError: "累積誤差", combineU: "u 合成", clamp: "制限", outputControl: "制御出力", applyToPlant: "プラントへ作用", positionError: "位置誤差", stateFeedback: "状態フィードバック"
});

Object.assign(I18N.ru, I18N.en, {
  appTitle: "Симулятор прогнозного и обратного управления", appSubtitle: "NMPC/MPC используют модельную оптимизацию со скользящим горизонтом; LQR/PID служат базовыми регуляторами обратной связи.",
  infoButton: "Справка", speed: "Скорость", pause: "Пауза", resume: "Продолжить", reset: "Сброс", metricX: "Позиция x", metricV: "Скорость v", metricU: "Управление u", metricCost: "Стоимость", metricCandidates: "Кандидаты",
  flowTitle: "Поток управления", waiting: "Ожидание данных", resetParams: "Сброс параметров", resetAll: "Сбросить все", sceneParams: "Параметры сцены", targetPosition: "Целевая позиция", drag: "Сопротивление",
  slowStart: "Начало ограничения", slowSpeed: "Лимит скорости", slowWeight: "Штраф лимита", predictiveParams: "Параметры прогноза", nmpcParams: "Параметры NMPC", mpcParams: "Параметры MPC", nmpcAdvancedParams: "Доп. параметры NMPC", mpcAdvancedParams: "Доп. параметры MPC", levels: "Уровни управления", posWeight: "Вес позиции", terminalWeight: "Терминальный вес", velocityWeight: "Вес скорости", smoothWeight: "Вес плавности",
  pidParams: "Параметры PID", kp: "Пропорц. Kp", ki: "Интегр. Ki", kd: "Дифф. Kd", integralLimit: "Лимит интеграла", lqrParams: "Параметры LQR", lqrKx: "Обратная связь Kx", lqrKv: "Обратная связь Kv",
  infoTitle: "Описание контроллеров", infoNmpc: "NMPC: прогнозирует будущее нелинейной моделью.", infoMpc: "MPC: прогнозирует будущее линейной моделью.", infoFeedback: "LQR/PID: не прогнозируют будущее, а реагируют на текущее состояние.", infoSlow: "Оранжевая зона: ограничение скорости, которое прогнозные контроллеры учитывают заранее.",
  running: "Работает", paused: "Пауза", stableRecording: "Запись стабильного хвоста", stable: "Стабильно", nonlinearModel: "Нелинейная модель", linearModel: "Линейная модель",
  modeNmpc: "NMPC нелинейный прогноз", modeMpc: "MPC линейный прогноз", modeLqr: "LQR обратная связь", modePid: "PID управление", slowZone: "Зона лимита", target: "Цель", error: "Ошибка", mode: "Режим",
  chartPosition: "Позиция", chartVelocity: "Скорость", chartControl: "Управление", chartTarget: "Цель", closedLoopLabel: "Замкнутый контур: новое состояние x,v возвращается на вход контроллера", stateData: "Данные состояния", candidateLoop: "Цикл кандидатов",
  flowFrozen: "Стабильный хвост завершен; поток управления остановлен", closedLoopShort: "Замкнутая связь", waitFirstSolve: "Ожидание первого решения", outputThenFeedback: "Замкнутый контур: выход и обратная связь",
  targetHold: "Удержание цели", lockedTarget: "x зафиксирован на цели", targetCapture: "Захват цели", dampingSettle: "Демпфирование", candidates: "Кандидаты", predictedTerminalX: "терминальный x", slowHits: "попадания лимита",
  readState: "Чтение состояния", generateCandidates: "Генерация", iterateCandidates: "Перебор", calcCost: "Расчет cost", saveBest: "Сохранить best", executeU0: "Выполнить u0", plantUpdate: "Обновить объект", feedbackNext: "Обратная связь",
  readError: "Чтение ошибки", accumulatedError: "накопленная ошибка", combineU: "Сумма u", clamp: "Ограничение", outputControl: "Выход", applyToPlant: "к объекту", positionError: "Ошибка позиции", stateFeedback: "обратная связь состояния"
});

let currentLang = "en";

function t(key) {
  return (I18N[currentLang] && I18N[currentLang][key]) || I18N["zh-CN"][key] || key;
}

const state = {
  x: 0,
  v: 0,
  u: 0,
  t: 0,
  previousU: 0,
  integral: 0,
  cost: 0,
  candidates: 0,
  flowPhase: 0,
  flowInfo: {},
  speedRemainder: 0,
  timeAxisMax: 10,
  timeTickStep: 2,
  stableSince: null,
  running: true,
  mode: "nmpc",
  history: []
};

const el = {};
for (const id of [
  "world", "chart", "flowNodes", "flowDetail", "pauseBtn", "resetBtn", "nmpcMode", "mpcMode", "lqrMode",
  "pidMode", "modeTitle",
  "statusText", "xValue", "vValue", "uValue", "costValue", "candValue", "target",
  "horizon", "levels", "posWeight", "terminalWeight", "velocityWeight",
  "smoothWeight", "drag", "slowStart", "slowSpeed", "slowWeight", "speed", "kp", "ki", "kd",
  "controlBlocks", "controlEffortWeight", "terminalVelocityMultiplier", "terminalPositionBoost",
  "terminalVelocityBoost", "settleRadius", "settleWeight", "holdError", "holdSpeed",
  "captureError", "captureSpeed", "captureKx", "captureKv", "captureLimit",
  "integralLimit", "lqrKx", "lqrKv", "resetParamsBtn", "resetAllBtn",
  "predictiveControls", "advancedPredictiveControls", "predictiveTitle", "advancedPredictiveTitle", "pidControls", "lqrControls", "infoBtn", "infoModal", "infoCloseBtn",
  "languageSelect"
]) {
  el[id] = document.getElementById(id);
}

const labels = {
  target: document.getElementById("targetLabel"),
  horizon: document.getElementById("horizonLabel"),
  levels: document.getElementById("levelsLabel"),
  posWeight: document.getElementById("posWeightLabel"),
  terminalWeight: document.getElementById("terminalWeightLabel"),
  velocityWeight: document.getElementById("velocityWeightLabel"),
  smoothWeight: document.getElementById("smoothWeightLabel"),
  controlBlocks: document.getElementById("controlBlocksLabel"),
  controlEffortWeight: document.getElementById("controlEffortWeightLabel"),
  terminalVelocityMultiplier: document.getElementById("terminalVelocityMultiplierLabel"),
  terminalPositionBoost: document.getElementById("terminalPositionBoostLabel"),
  terminalVelocityBoost: document.getElementById("terminalVelocityBoostLabel"),
  settleRadius: document.getElementById("settleRadiusLabel"),
  settleWeight: document.getElementById("settleWeightLabel"),
  holdError: document.getElementById("holdErrorLabel"),
  holdSpeed: document.getElementById("holdSpeedLabel"),
  captureError: document.getElementById("captureErrorLabel"),
  captureSpeed: document.getElementById("captureSpeedLabel"),
  captureKx: document.getElementById("captureKxLabel"),
  captureKv: document.getElementById("captureKvLabel"),
  captureLimit: document.getElementById("captureLimitLabel"),
  drag: document.getElementById("dragLabel"),
  slowStart: document.getElementById("slowStartLabel"),
  slowSpeed: document.getElementById("slowSpeedLabel"),
  slowWeight: document.getElementById("slowWeightLabel"),
  speed: document.getElementById("speedLabel"),
  kp: document.getElementById("kpLabel"),
  ki: document.getElementById("kiLabel"),
  kd: document.getElementById("kdLabel"),
  integralLimit: document.getElementById("integralLimitLabel"),
  lqrKx: document.getElementById("lqrKxLabel"),
  lqrKv: document.getElementById("lqrKvLabel")
};

function applyLanguage(lang) {
  currentLang = I18N[lang] ? lang : "zh-CN";
  document.documentElement.lang = currentLang;
  el.languageSelect.value = currentLang;
  state.flowInfo = {};
  state.flowPhase = 0;
  try {
    localStorage.setItem("nmpcDemoLang", currentLang);
  } catch (_) {
    // Ignore storage failures in restricted browser contexts.
  }
  document.title = t("appTitle");
  document.querySelectorAll("[data-i18n]").forEach(node => {
    node.textContent = t(node.dataset.i18n);
  });
  updateLabels();
  renderMetrics();
  drawWorld();
  drawChart();
  renderFlow();
}

function initialLanguage() {
  const queryLang = new URLSearchParams(window.location.search).get("lang");
  if (I18N[queryLang]) return queryLang;
  try {
    const stored = localStorage.getItem("nmpcDemoLang");
    if (I18N[stored]) return stored;
  } catch (_) {
    // Fall back to Simplified Chinese.
  }
  return "en";
}

function openInfoModal() {
  el.infoModal.hidden = false;
  el.infoCloseBtn.focus();
}

function closeInfoModal() {
  el.infoModal.hidden = true;
  el.infoBtn.focus();
}

function params() {
  return {
    target: Number(el.target.value),
    horizon: Number(el.horizon.value),
    levelCount: Number(el.levels.value),
    posWeight: Number(el.posWeight.value),
    terminalWeight: Number(el.terminalWeight.value),
    velocityWeight: Number(el.velocityWeight.value),
    smoothWeight: Number(el.smoothWeight.value),
    controlBlocks: Number(el.controlBlocks.value),
    controlEffortWeight: Number(el.controlEffortWeight.value),
    terminalVelocityMultiplier: Number(el.terminalVelocityMultiplier.value),
    terminalPositionBoost: Number(el.terminalPositionBoost.value),
    terminalVelocityBoost: Number(el.terminalVelocityBoost.value),
    settleRadius: Number(el.settleRadius.value),
    settleWeight: Number(el.settleWeight.value),
    holdError: Number(el.holdError.value),
    holdSpeed: Number(el.holdSpeed.value),
    captureError: Number(el.captureError.value),
    captureSpeed: Number(el.captureSpeed.value),
    captureKx: Number(el.captureKx.value),
    captureKv: Number(el.captureKv.value),
    captureLimit: Number(el.captureLimit.value),
    drag: Number(el.drag.value),
    slowStart: Number(el.slowStart.value),
    slowSpeed: Number(el.slowSpeed.value),
    slowWeight: Number(el.slowWeight.value),
    speed: Number(el.speed.value),
    kp: Number(el.kp.value),
    ki: Number(el.ki.value),
    kd: Number(el.kd.value),
    integralLimit: Number(el.integralLimit.value),
    lqrKx: Number(el.lqrKx.value),
    lqrKv: Number(el.lqrKv.value)
  };
}

function updateLabels() {
  const p = params();
  labels.target.textContent = p.target.toFixed(1);
  labels.horizon.textContent = String(p.horizon);
  labels.levels.textContent = String(p.levelCount);
  labels.posWeight.textContent = p.posWeight.toFixed(1);
  labels.terminalWeight.textContent = p.terminalWeight.toFixed(1);
  labels.velocityWeight.textContent = p.velocityWeight.toFixed(2);
  labels.smoothWeight.textContent = p.smoothWeight.toFixed(2);
  labels.controlBlocks.textContent = String(p.controlBlocks);
  labels.controlEffortWeight.textContent = p.controlEffortWeight.toFixed(2);
  labels.terminalVelocityMultiplier.textContent = p.terminalVelocityMultiplier.toFixed(1);
  labels.terminalPositionBoost.textContent = p.terminalPositionBoost.toFixed(1);
  labels.terminalVelocityBoost.textContent = p.terminalVelocityBoost.toFixed(1);
  labels.settleRadius.textContent = p.settleRadius.toFixed(2);
  labels.settleWeight.textContent = p.settleWeight.toFixed(1);
  labels.holdError.textContent = p.holdError.toFixed(3);
  labels.holdSpeed.textContent = p.holdSpeed.toFixed(3);
  labels.captureError.textContent = p.captureError.toFixed(2);
  labels.captureSpeed.textContent = p.captureSpeed.toFixed(2);
  labels.captureKx.textContent = p.captureKx.toFixed(1);
  labels.captureKv.textContent = p.captureKv.toFixed(1);
  labels.captureLimit.textContent = p.captureLimit.toFixed(2);
  labels.drag.textContent = p.drag.toFixed(2);
  labels.slowStart.textContent = p.slowStart.toFixed(1);
  labels.slowSpeed.textContent = p.slowSpeed.toFixed(2);
  labels.slowWeight.textContent = p.slowWeight.toFixed(1);
  labels.speed.textContent = `${p.speed.toFixed(2)}x`;
  labels.kp.textContent = p.kp.toFixed(2);
  labels.ki.textContent = p.ki.toFixed(2);
  labels.kd.textContent = p.kd.toFixed(2);
  labels.integralLimit.textContent = p.integralLimit.toFixed(1);
  labels.lqrKx.textContent = p.lqrKx.toFixed(2);
  labels.lqrKv.textContent = p.lqrKv.toFixed(2);
}

const PRESETS = {
  common: {
    target: "10",
    drag: "0.08",
    slowStart: "7",
    slowSpeed: "0.8",
    slowWeight: "16",
    speed: "1"
  },
  predictiveAdvanced: {
    controlBlocks: "3",
    controlEffortWeight: "0.04",
    terminalVelocityMultiplier: "8",
    terminalPositionBoost: "4.5",
    terminalVelocityBoost: "18",
    settleRadius: "0.75",
    settleWeight: "7.5",
    holdError: "0.035",
    holdSpeed: "0.045",
    captureError: "0.42",
    captureSpeed: "0.55",
    captureKx: "5.4",
    captureKv: "3.4",
    captureLimit: "0.5"
  },
  nmpc: {
    horizon: "28",
    levels: "7",
    posWeight: "1.2",
    terminalWeight: "13",
    velocityWeight: "0.34",
    smoothWeight: "0.34"
  },
  mpc: {
    horizon: "32",
    levels: "7",
    posWeight: "0.9",
    terminalWeight: "16",
    velocityWeight: "0.58",
    smoothWeight: "0.42"
  },
  pid: {
    kp: "0.34",
    ki: "0.04",
    kd: "0.62",
    integralLimit: "6"
  },
  lqr: {
    lqrKx: "0.32",
    lqrKv: "0.86"
  }
};

function presetForMode(mode) {
  if (mode === "nmpc") return { ...PRESETS.common, ...PRESETS.predictiveAdvanced, ...PRESETS.nmpc };
  if (mode === "mpc") return { ...PRESETS.common, ...PRESETS.predictiveAdvanced, ...PRESETS.mpc };
  if (mode === "pid") return { ...PRESETS.common, ...PRESETS.pid };
  if (mode === "lqr") return { ...PRESETS.common, ...PRESETS.lqr };
  return { ...PRESETS.common, ...PRESETS.nmpc };
}

function resetParams() {
  const defaults = presetForMode(state.mode);
  for (const [id, value] of Object.entries(defaults)) {
    el[id].value = value;
  }
  updateLabels();
}

function resetAll() {
  resetParams();
  reset();
}

function clamp(value, low, high) {
  return Math.max(low, Math.min(high, value));
}

function plantStep(s, u, p) {
  const limitedU = clamp(u, -1, 1);
  const a = limitedU - p.drag * s.v * Math.abs(s.v);
  const v = clamp(s.v + a * DT, -MAX_SPEED, MAX_SPEED);
  const x = s.x + v * DT;
  return { x, v };
}

function linearModelStep(s, u) {
  const limitedU = clamp(u, -1, 1);
  const v = clamp(s.v + limitedU * DT, -MAX_SPEED, MAX_SPEED);
  const x = s.x + v * DT;
  return { x, v };
}

function controlLevels(count) {
  const levels = [];
  const mid = (count - 1) / 2;
  for (let i = 0; i < count; i++) {
    levels.push((i - mid) / mid);
  }
  return levels;
}

function expandBlocks(blocks, horizon) {
  const blockLength = Math.max(1, Math.floor(horizon / blocks.length));
  const controls = [];
  for (const u of blocks) {
    for (let i = 0; i < blockLength; i++) controls.push(u);
  }
  while (controls.length < horizon) controls.push(blocks[blocks.length - 1]);
  return controls.slice(0, horizon);
}

function rollout(start, controls, previousU, p, stepModel) {
  let s = { x: start.x, v: start.v };
  let cost = 0;
  let lastU = previousU;
  let slowPenaltyHit = false;
  for (let i = 0; i < controls.length; i++) {
    const u = controls[i];
    s = stepModel(s, u, p);
    const error = s.x - p.target;
    const terminal = i === controls.length - 1;
    const positionWeight = terminal ? p.terminalWeight : p.posWeight;
    const velocityWeight = terminal ? p.velocityWeight * p.terminalVelocityMultiplier : p.velocityWeight;
    cost += positionWeight * error * error;
    cost += velocityWeight * s.v * s.v;
    if (Math.abs(error) < p.settleRadius) {
      const settleWeight = p.settleWeight * (p.settleRadius - Math.abs(error));
      cost += settleWeight * s.v * s.v;
    }
    if (s.x >= p.slowStart && Math.abs(s.v) > p.slowSpeed) {
      const excessSpeed = Math.abs(s.v) - p.slowSpeed;
      cost += p.slowWeight * excessSpeed * excessSpeed;
      slowPenaltyHit = true;
    }
    cost += p.controlEffortWeight * u * u;
    cost += p.smoothWeight * (u - lastU) * (u - lastU);
    lastU = u;
  }
  const terminalError = s.x - p.target;
  cost += p.terminalWeight * p.terminalPositionBoost * terminalError * terminalError;
  cost += (p.terminalWeight + p.velocityWeight * p.terminalVelocityBoost) * s.v * s.v;
  return { terminal: s, cost, slowPenaltyHit };
}

function searchControlBlocks(levels, blockCount, visit) {
  const blocks = new Array(blockCount);
  function walk(depth) {
    if (depth === blockCount) {
      visit(blocks);
      return;
    }
    for (const level of levels) {
      blocks[depth] = level;
      walk(depth + 1);
    }
  }
  walk(0);
}

function solveShooting(p, stepModel, modelName) {
  const hold = predictiveHoldSolution(p, modelName);
  if (hold) return hold;
  const levels = controlLevels(p.levelCount);
  const blockCount = Math.max(1, Math.round(p.controlBlocks));
  let bestU = 0;
  let bestCost = Infinity;
  let bestTerminal = { x: state.x, v: state.v };
  let candidates = 0;
  let slowPenaltyHits = 0;

  searchControlBlocks(levels, blockCount, blocks => {
    const controls = expandBlocks(blocks, p.horizon);
    const result = rollout(
      { x: state.x, v: state.v },
      controls,
      state.previousU,
      p,
      stepModel
    );
    candidates += 1;
    if (result.slowPenaltyHit) {
      slowPenaltyHits += 1;
    }
    if (result.cost < bestCost) {
      bestCost = result.cost;
      bestU = blocks[0];
      bestTerminal = result.terminal;
    }
  });

  if (!Number.isFinite(bestCost)) {
    bestCost = 0;
    bestU = 0;
    bestTerminal = { x: state.x, v: state.v };
  }

  state.flowInfo = {
    modelName,
    bestCost,
    bestU,
    candidates,
    terminalX: bestTerminal.x,
    terminalV: bestTerminal.v,
    slowPenaltyHits
  };
  return { u: bestU, cost: bestCost, candidates, terminal: bestTerminal };
}

function predictiveHoldSolution(p, modelName) {
  const error = p.target - state.x;
  const nearTarget = Math.abs(error) <= p.holdError && Math.abs(state.v) <= p.holdSpeed;
  if (nearTarget) {
    state.flowInfo = {
      modelName,
      bestCost: 0,
      bestU: 0,
      candidates: 1,
      terminalX: p.target,
      terminalV: 0,
      slowPenaltyHits: 0,
      holding: true
    };
    return { u: 0, cost: 0, candidates: 1, terminal: { x: p.target, v: 0 }, holding: true };
  }

  if (Math.abs(error) > p.captureError || Math.abs(state.v) > p.captureSpeed) return null;
  const u = clamp(p.captureKx * error - p.captureKv * state.v, -p.captureLimit, p.captureLimit);
  const cost = error * error + 0.8 * state.v * state.v + p.controlEffortWeight * u * u;
  state.flowInfo = {
    modelName,
    bestCost: cost,
    bestU: u,
    candidates: 1,
    terminalX: state.x,
    terminalV: state.v,
    slowPenaltyHits: 0,
    capturing: true
  };
  return { u, cost, candidates: 1, terminal: { x: state.x, v: state.v }, capturing: true };
}

function solveNmpc(p) {
  return solveShooting(p, plantStep, t("nonlinearModel"));
}

function solveMpc(p) {
  return solveShooting(p, linearModelStep, t("linearModel"));
}

function solvePid(p) {
  const error = p.target - state.x;
  state.integral = clamp(
    state.integral + error * DT,
    -p.integralLimit,
    p.integralLimit
  );
  const u = clamp(p.kp * error + p.ki * state.integral - p.kd * state.v, -1, 1);
  state.flowInfo = {
    pTerm: p.kp * error,
    iTerm: p.ki * state.integral,
    dTerm: -p.kd * state.v,
    integral: state.integral
  };
  const cost =
    error * error +
    0.4 * state.v * state.v +
    0.05 * state.integral * state.integral +
    0.1 * u * u;
  return { u, cost, candidates: 1, terminal: { x: state.x, v: state.v } };
}

function solveLqr(p) {
  const error = state.x - p.target;
  const u = clamp(-p.lqrKx * error - p.lqrKv * state.v, -1, 1);
  state.flowInfo = {
    xTerm: -p.lqrKx * error,
    vTerm: -p.lqrKv * state.v,
    error
  };
  const cost = error * error + 0.4 * state.v * state.v + 0.1 * u * u;
  return { u, cost, candidates: 1, terminal: { x: state.x, v: state.v } };
}

function step() {
  const p = params();
  const start = { x: state.x, v: state.v };
  const solution =
    state.mode === "nmpc" ? solveNmpc(p) :
    state.mode === "mpc" ? solveMpc(p) :
    state.mode === "lqr" ? solveLqr(p) :
    solvePid(p);
  const next = solution.holding ? { x: p.target, v: 0 } : plantStep(start, solution.u, p);
  state.x = next.x;
  state.v = next.v;
  state.u = solution.u;
  state.previousU = solution.u;
  state.cost = solution.cost;
  state.candidates = solution.candidates;
  state.flowPhase = (state.flowPhase + 1) % FLOW_PHASE_COUNT;
  if (shouldRecordSample(p)) {
    state.t += DT;
    state.history.push({ t: state.t, x: state.x, v: state.v, u: state.u, target: p.target });
  }
}

function shouldRecordSample(p) {
  if (!isStableAtTarget(p)) {
    state.stableSince = null;
    return true;
  }
  if (state.stableSince === null) {
    state.stableSince = state.t;
  }
  return state.t - state.stableSince < STABLE_TAIL_SECONDS;
}

function isStableTailComplete(p) {
  return isStableAtTarget(p) &&
    state.stableSince !== null &&
    state.t - state.stableSince >= STABLE_TAIL_SECONDS;
}

function isStableAtTarget(p) {
  return Math.abs(p.target - state.x) <= STABLE_ERROR &&
    Math.abs(state.v) <= STABLE_SPEED &&
    Math.abs(state.u) <= STABLE_CONTROL;
}

function reset() {
  state.x = 0;
  state.v = 0;
  state.u = 0;
  state.t = 0;
  state.previousU = 0;
  state.integral = 0;
  state.cost = 0;
  state.candidates = 0;
  state.flowPhase = 0;
  state.flowInfo = {};
  state.speedRemainder = 0;
  state.timeAxisMax = 10;
  state.timeTickStep = 2;
  state.stableSince = null;
  state.history = [];
}

function resizeCanvas(canvas) {
  const rect = canvas.getBoundingClientRect();
  const dpr = window.devicePixelRatio || 1;
  canvas.width = Math.max(1, Math.floor(rect.width * dpr));
  canvas.height = Math.max(1, Math.floor(rect.height * dpr));
  const ctx = canvas.getContext("2d");
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  return { ctx, width: rect.width, height: rect.height };
}

function drawWorld() {
  const { ctx, width, height } = resizeCanvas(el.world);
  const p = params();
  ctx.clearRect(0, 0, width, height);
  ctx.fillStyle = "#f8faf8";
  ctx.fillRect(0, 0, width, height);

  const minX = -13;
  const maxX = 13;
  const toPx = x => 48 + (x - minX) / (maxX - minX) * (width - 96);
  const trackY = height * 0.58;

  ctx.strokeStyle = "#cfdad5";
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.moveTo(48, trackY);
  ctx.lineTo(width - 48, trackY);
  ctx.stroke();

  for (let x = -12; x <= 12; x += 4) {
    const px = toPx(x);
    ctx.strokeStyle = "#d9e2dd";
    ctx.beginPath();
    ctx.moveTo(px, trackY - 8);
    ctx.lineTo(px, trackY + 8);
    ctx.stroke();
    ctx.fillStyle = "#66736f";
    ctx.font = "12px Arial";
    ctx.textAlign = "center";
    ctx.fillText(String(x), px, trackY + 28);
  }

  const targetX = toPx(p.target);
  const slowX = toPx(p.slowStart);
  ctx.fillStyle = "rgba(194, 106, 27, 0.10)";
  ctx.fillRect(slowX, 18, width - 48 - slowX, height - 52);
  ctx.fillStyle = "#9a5a18";
  ctx.font = "700 13px Arial";
  ctx.textAlign = "left";
  ctx.fillText(`${t("slowZone")} |v| <= ${p.slowSpeed.toFixed(2)}`, slowX + 8, height - 40);

  ctx.strokeStyle = "#b43f35";
  ctx.lineWidth = 3;
  ctx.beginPath();
  ctx.moveTo(targetX, 24);
  ctx.lineTo(targetX, height - 34);
  ctx.stroke();
  ctx.fillStyle = "#b43f35";
  ctx.font = "700 13px Arial";
  ctx.textAlign = "center";
  ctx.fillText(t("target"), targetX, 16);

  const carX = toPx(state.x);
  ctx.fillStyle = "#2563a6";
  ctx.strokeStyle = "#174e84";
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.roundRect(carX - 21, trackY - 29, 42, 24, 5);
  ctx.fill();
  ctx.stroke();
  ctx.fillStyle = "#172d29";
  ctx.beginPath();
  ctx.arc(carX - 13, trackY, 5, 0, Math.PI * 2);
  ctx.arc(carX + 13, trackY, 5, 0, Math.PI * 2);
  ctx.fill();

  ctx.fillStyle = "#344844";
  ctx.font = "13px Arial";
  ctx.textAlign = "left";
  ctx.fillText(`${t("error")} = ${(p.target - state.x).toFixed(2)}`, 18, 24);
  ctx.fillText(`${t("mode")} = ${modeName()}`, 18, 44);
}

function drawChart() {
  const { ctx, width, height } = resizeCanvas(el.chart);
  ctx.clearRect(0, 0, width, height);
  ctx.fillStyle = "#ffffff";
  ctx.fillRect(0, 0, width, height);

  const pad = { left: 86, right: 20, top: 22, bottom: 34 };
  const chartW = width - pad.left - pad.right;
  const chartH = height - pad.top - pad.bottom;
  const samples = state.history;
  const axis = autoAxis(samples);

  const timeAxis = autoTimeAxis(samples);
  const xScale = sample => {
    if (timeAxis.max <= timeAxis.min) return pad.left;
    return pad.left + (sample.t - timeAxis.min) / (timeAxis.max - timeAxis.min) * chartW;
  };
  const yScale = value => pad.top + (axis.max - value) / (axis.max - axis.min) * chartH;

  ctx.strokeStyle = "#edf1ef";
  ctx.lineWidth = 1;
  for (const tick of axis.ticks) {
    const y = yScale(tick);
    ctx.beginPath();
    ctx.moveTo(pad.left, y);
    ctx.lineTo(width - pad.right, y);
    ctx.stroke();
  }
  for (const tick of timeAxis.ticks) {
    const x = pad.left + (tick - timeAxis.min) / (timeAxis.max - timeAxis.min) * chartW;
    ctx.beginPath();
    ctx.moveTo(x, pad.top);
    ctx.lineTo(x, pad.top + chartH);
    ctx.stroke();
  }

  function drawSeries(key, color) {
    if (samples.length < 2) return;
    const stride = Math.max(1, Math.floor(samples.length / chartW));
    ctx.strokeStyle = color;
    ctx.lineWidth = 2;
    ctx.beginPath();
    samples.forEach((sample, i) => {
      if (i !== 0 && i !== samples.length - 1 && i % stride !== 0) return;
      const x = xScale(sample);
      const y = yScale(sample[key]);
      if (i === 0) ctx.moveTo(x, y);
      else ctx.lineTo(x, y);
    });
    ctx.stroke();
  }

  drawSeries("target", "#b43f35");
  drawSeries("x", "#2563a6");
  drawSeries("v", "#2f855a");
  drawSeries("u", "#c26a1b");

  if (axis.min < 0 && axis.max > 0) {
    const zeroY = yScale(0);
    ctx.strokeStyle = "#d6e0db";
    ctx.lineWidth = 1.2;
    ctx.beginPath();
    ctx.moveTo(pad.left, zeroY);
    ctx.lineTo(width - pad.right, zeroY);
    ctx.stroke();
  }

  ctx.fillStyle = "#66736f";
  ctx.font = "11px Arial";
  ctx.textAlign = "right";
  for (const tick of axis.ticks) {
    const y = yScale(tick);
    ctx.fillText(formatAxis(tick), pad.left - 10, y + 4);
  }
  ctx.textAlign = "center";
  for (const tick of timeAxis.ticks) {
    const x = pad.left + (tick - timeAxis.min) / (timeAxis.max - timeAxis.min) * chartW;
    ctx.fillText(`${formatTime(tick)}s`, x, pad.top + chartH + 18);
  }
  ctx.textAlign = "left";
  ctx.fillText(`y: ${formatAxis(axis.min)} .. ${formatAxis(axis.max)} | t: ${formatTime(timeAxis.min)}s .. ${formatTime(timeAxis.max)}s`, pad.left + 6, pad.top + 14);
  ctx.fillText("t", width - pad.right - 6, pad.top + chartH + 30);

  ctx.strokeStyle = "#cfdad5";
  ctx.strokeRect(pad.left, pad.top, chartW, chartH);
  drawChartLegend(ctx, width - pad.right - 190, pad.top + 10);
}

function drawChartLegend(ctx, x, y) {
  const items = [
    [t("chartPosition"), "#2563a6"],
    [t("chartVelocity"), "#2f855a"],
    [t("chartControl"), "#c26a1b"],
    [t("chartTarget"), "#b43f35"]
  ];
  const width = 170;
  const height = 54;
  ctx.save();
  ctx.fillStyle = "rgba(255, 255, 255, 0.88)";
  ctx.strokeStyle = "#d7e0dc";
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.roundRect(x, y, width, height, 7);
  ctx.fill();
  ctx.stroke();
  ctx.font = "11px Arial";
  ctx.textAlign = "left";
  ctx.textBaseline = "middle";
  items.forEach((item, index) => {
    const col = index % 2;
    const row = Math.floor(index / 2);
    const itemX = x + 12 + col * 82;
    const itemY = y + 17 + row * 20;
    ctx.strokeStyle = item[1];
    ctx.lineWidth = 2.5;
    ctx.beginPath();
    ctx.moveTo(itemX, itemY);
    ctx.lineTo(itemX + 18, itemY);
    ctx.stroke();
    ctx.fillStyle = "#53625e";
    ctx.fillText(item[0], itemX + 26, itemY);
  });
  ctx.restore();
}

function autoAxis(samples) {
  let min = -1;
  let max = 1;
  const keys = ["target", "x", "v", "u"];
  for (const sample of samples) {
    for (const key of keys) {
      const value = sample[key];
      if (!Number.isFinite(value)) continue;
      min = Math.min(min, value);
      max = Math.max(max, value);
    }
  }
  if (Math.abs(max - min) < 1e-6) {
    min -= 1;
    max += 1;
  }
  const span = max - min;
  min -= span * 0.12;
  max += span * 0.12;
  const niceMin = niceFloor(min);
  const niceMax = niceCeil(max);
  if (niceMax <= niceMin) return { min: -1, max: 1, ticks: [-1, 0, 1] };
  return { min: niceMin, max: niceMax, ticks: makeTicks(niceMin, niceMax, 5) };
}

function autoTimeAxis(samples) {
  const min = 0;
  const latest = samples.length < 2 ? 0 : samples[samples.length - 1].t;
  const desiredMax = Math.max(10, latest * 1.06 + 0.25);
  if (!Number.isFinite(state.timeAxisMax) || state.timeAxisMax < desiredMax - 4) {
    state.timeAxisMax = desiredMax;
  } else {
    state.timeAxisMax += (desiredMax - state.timeAxisMax) * 0.045;
  }
  const max = Math.max(10, state.timeAxisMax);
  const targetStep = niceStep(max / 5);
  const currentStep = state.timeTickStep || targetStep;
  if (targetStep > currentStep && max / currentStep > 7.5) {
    state.timeTickStep = targetStep;
  } else if (targetStep < currentStep && max / targetStep < 4.2) {
    state.timeTickStep = targetStep;
  }
  const step = state.timeTickStep || targetStep;
  return { min, max, ticks: makeTicks(min, max, 6, step) };
}

function makeTicks(min, max, desiredCount, fixedStep) {
  const span = Math.max(1e-9, max - min);
  const step = fixedStep || niceStep(span / Math.max(1, desiredCount - 1));
  const first = Math.ceil(min / step) * step;
  const ticks = [];
  for (let value = first; value <= max + step * 0.5; value += step) {
    if (value >= min - step * 0.5 && value <= max + step * 0.5) {
      ticks.push(Math.abs(value) < 1e-12 ? 0 : value);
    }
  }
  if (ticks.length < 2) return [min, max];
  return ticks.slice(0, 8);
}

function niceStep(value) {
  const magnitude = Math.pow(10, Math.floor(Math.log10(Math.max(1e-9, Math.abs(value)))));
  const normalized = Math.abs(value) / magnitude;
  if (normalized <= 1) return magnitude;
  if (normalized <= 2) return 2 * magnitude;
  if (normalized <= 5) return 5 * magnitude;
  return 10 * magnitude;
}

function niceFloor(value) {
  const step = niceStep(value / 4);
  return Math.floor(value / step) * step;
}

function niceCeil(value) {
  const step = niceStep(value / 4);
  return Math.ceil(value / step) * step;
}

function formatAxis(value) {
  const abs = Math.abs(value);
  if (abs >= 100) return value.toFixed(0);
  if (abs >= 10) return value.toFixed(1);
  return value.toFixed(2);
}

function formatTime(value) {
  if (Math.abs(value) >= 100) return value.toFixed(0);
  if (Math.abs(value) >= 10) return value.toFixed(1);
  return value.toFixed(2);
}

function flowForMode(mode) {
  if (mode === "nmpc" || mode === "mpc") {
    const model = mode === "nmpc" ? t("nonlinearModel") : t("linearModel");
    return {
      detail: predictiveFlowDetail(model),
      nodes: [
        ["input", t("readState"), t("stateVars")],
        ["loop", t("generateCandidates"), t("segmentedControl")],
        ["loop", "ForLoop", t("iterateCandidates")],
        ["input", "rollout", model],
        ["decision", t("calcCost"), t("costTerms")],
        ["decision", t("saveBest"), t("costLower")],
        ["output", t("executeU0"), t("firstStepOnly")],
        ["plant", t("plantUpdate"), t("stateChanged")],
        ["feedback", t("feedbackNext"), t("rereadState")]
      ]
    };
  }
  if (mode === "pid") {
    return {
      detail: pidFlowDetail(),
      nodes: [
        ["input", t("readError"), "target - x"],
        ["input", t("pTerm"), "Kp * error"],
        ["input", t("iTerm"), t("accumulatedError")],
        ["input", t("dTerm"), "-Kd * v"],
        ["decision", t("combineU"), t("pidSum")],
        ["limit", t("clamp"), t("uRange")],
        ["output", t("outputControl"), t("applyToPlant")],
        ["plant", t("plantUpdate"), t("stateChanged")],
        ["feedback", t("feedbackNext"), t("rereadError")]
      ]
    };
  }
  return {
    detail: lqrFlowDetail(),
    nodes: [
      ["input", t("readState"), t("stateVars")],
      ["input", t("positionError"), "x - target"],
      ["input", t("kxFeedback"), "-Kx * error"],
      ["input", t("kvFeedback"), "-Kv * v"],
      ["decision", t("combineU"), t("stateFeedback")],
      ["limit", t("clamp"), t("uRange")],
      ["output", t("outputControl"), t("applyToPlant")],
      ["plant", t("plantUpdate"), t("stateChanged")],
      ["feedback", t("feedbackNext"), t("rereadState")]
    ]
  };
}

function predictiveFlowDetail(model) {
  const info = state.flowInfo || {};
  if (!Number.isFinite(info.bestCost)) return `${model} | ${t("outputThenFeedback")} | ${t("waitFirstSolve")}`;
  if (info.holding) return `${model} | ${t("targetHold")} | ${t("lockedTarget")} | v 0.00 | u0 0.00 | ${t("closedLoopShort")}`;
  if (info.capturing) return `${model} | ${t("targetCapture")} | ${t("dampingSettle")} | u0 ${info.bestU.toFixed(2)} | ${t("closedLoopShort")}`;
  return `${model} | ${t("candidates")} ${info.candidates} | best cost ${info.bestCost.toFixed(2)} | ` +
    `u0 ${info.bestU.toFixed(2)} | ${t("predictedTerminalX")} ${info.terminalX.toFixed(2)} | ` +
    `${t("slowHits")} ${info.slowPenaltyHits} | ${t("closedLoopShort")}`;
}

function pidFlowDetail() {
  const info = state.flowInfo || {};
  const pTerm = Number.isFinite(info.pTerm) ? info.pTerm.toFixed(2) : "0.00";
  const iTerm = Number.isFinite(info.iTerm) ? info.iTerm.toFixed(2) : "0.00";
  const dTerm = Number.isFinite(info.dTerm) ? info.dTerm.toFixed(2) : "0.00";
  return `P ${pTerm} | I ${iTerm} | D ${dTerm} | integral ${(info.integral || 0).toFixed(2)} | ${t("closedLoopShort")}`;
}

function lqrFlowDetail() {
  const info = state.flowInfo || {};
  const xTerm = Number.isFinite(info.xTerm) ? info.xTerm.toFixed(2) : "0.00";
  const vTerm = Number.isFinite(info.vTerm) ? info.vTerm.toFixed(2) : "0.00";
  return `${t("kxTerm")} ${xTerm} | ${t("kvTerm")} ${vTerm} | error ${(info.error || 0).toFixed(2)} | ${t("closedLoopShort")}`;
}

function escapeHtml(value) {
  return String(value)
    .replaceAll("&", "&amp;")
    .replaceAll("<", "&lt;")
    .replaceAll(">", "&gt;")
    .replaceAll('"', "&quot;");
}

function renderFlow() {
  const flow = flowForMode(state.mode);
  const p = params();
  const frozen = isStableTailComplete(p);
  el.flowDetail.textContent = frozen ? `${flow.detail} | ${t("flowFrozen")}` : flow.detail;
  el.flowNodes.innerHTML = flowSvg(flow.nodes);
}

function flowSvg(nodes) {
  const rect = el.flowNodes.getBoundingClientRect();
  const vw = Math.max(1120, Math.floor(rect.width || 1120));
  const vh = Math.max(160, Math.floor(rect.height || 160));
  const side = 28;
  const topY = Math.max(22, vh * 0.16);
  const bottomY = Math.min(vh - 50, Math.max(102, vh * 0.66));
  const nodeW = clamp((vw - side * 2 - 6 * 26) / 7, 116, 160);
  const nodeH = 46;
  const availableGap = (vw - side * 2 - nodeW * 7) / 6;
  const gap = Math.max(24, availableGap);
  const topNodes = nodes.slice(0, 7).map((node, index) => ({
    node,
    index,
    x: side + index * (nodeW + gap),
    y: topY,
    w: nodeW,
    h: nodeH
  }));
  const plantW = Math.min(170, Math.max(142, nodeW + 12));
  const feedbackW = Math.min(190, Math.max(164, nodeW + 28));
  const plant = {
    node: nodes[7],
    index: 7,
    x: Math.min(vw - side - plantW, topNodes[6].x + nodeW * 0.42),
    y: bottomY,
    w: plantW,
    h: 42
  };
  const feedback = {
    node: nodes[8],
    index: 8,
    x: Math.max(side + 210, Math.min(vw - side - plantW - feedbackW - 80, vw * 0.38)),
    y: bottomY,
    w: feedbackW,
    h: 42
  };
  const all = [...topNodes, plant, feedback];
  const execLinks = [];
  for (let i = 0; i < topNodes.length - 1; i++) {
    execLinks.push(flowLine(topNodes[i], topNodes[i + 1], "exec"));
  }
  execLinks.push(flowPath(
    `M ${topNodes[6].x + topNodes[6].w} ${topNodes[6].y + nodeH / 2} ` +
    `C ${topNodes[6].x + topNodes[6].w + 42} ${topY + 24}, ` +
    `${plant.x - 58} ${bottomY + 20}, ${plant.x} ${plant.y + 21}`,
    "exec"
  ));

  const feedbackPath =
    `M ${plant.x} ${plant.y + 32} ` +
    `C ${plant.x - 95} ${vh - 12}, ${feedback.x + feedback.w + 110} ${vh - 12}, ${feedback.x + feedback.w} ${feedback.y + 30} ` +
    `L ${feedback.x} ${feedback.y + 30} ` +
    `C ${side + 210} ${vh - 12}, ${side + 74} ${vh - 12}, ${topNodes[0].x + 32} ${topNodes[0].y + topNodes[0].h}`;

  const dataPath =
    `M ${topNodes[0].x + 76} ${topNodes[0].y + topNodes[0].h} ` +
    `C ${topNodes[0].x + 190} ${topY + 72}, ${topNodes[2].x - 34} ${topY + 72}, ${topNodes[2].x + 18} ${topNodes[2].y + topNodes[2].h}`;

  const loopPath =
    `M ${topNodes[2].x + 70} ${topNodes[2].y + topNodes[2].h} ` +
    `C ${topNodes[2].x + 120} ${topY + 76}, ${topNodes[3].x + 16} ${topY + 76}, ${topNodes[3].x + 68} ${topNodes[3].y + topNodes[3].h}`;

  return `
    <svg class="flow-svg" viewBox="0 0 ${vw} ${vh}" preserveAspectRatio="none" role="img" aria-label="${escapeHtml(t("flowTitle"))}">
      <defs>
        <marker id="arrowExec" markerWidth="8" markerHeight="8" refX="7" refY="4" orient="auto">
          <path d="M 0 0 L 8 4 L 0 8 z" fill="#9eaca7"></path>
        </marker>
        <marker id="arrowData" markerWidth="8" markerHeight="8" refX="7" refY="4" orient="auto">
          <path d="M 0 0 L 8 4 L 0 8 z" fill="#2f855a"></path>
        </marker>
        <marker id="arrowFeedback" markerWidth="8" markerHeight="8" refX="7" refY="4" orient="auto">
          <path d="M 0 0 L 8 4 L 0 8 z" fill="#2563a6"></path>
        </marker>
      </defs>
      ${execLinks.join("")}
      ${flowPath(dataPath, "data")}
      ${flowPath(loopPath, "loop")}
      ${flowPath(feedbackPath, "feedback")}
      <text class="flow-label" x="${Math.min(vw - 420, Math.max(side + 520, feedback.x + feedback.w + 130))}" y="${vh - 18}">${escapeHtml(t("closedLoopLabel"))}</text>
      <text class="flow-label" x="${topNodes[1].x + nodeW * 0.70}" y="${topY + 78}">${escapeHtml(t("stateData"))}</text>
      <text class="flow-label" x="${topNodes[2].x + nodeW * 1.24}" y="${topY + 79}">${escapeHtml(t("candidateLoop"))}</text>
      ${topNodes.map((item, index) => flowBadge(index + 1, item.x - 9, item.y - 7)).join("")}
      ${flowBadge(8, plant.x - 9, plant.y - 7)}
      ${flowBadge(9, feedback.x - 9, feedback.y - 7)}
      ${all.map(flowNodeSvg).join("")}
    </svg>
  `;
}

function flowNodeSvg(item) {
  const [kind, title, text] = item.node;
  const active = item.index === state.flowPhase ? " active" : "";
  const visited = item.index <= state.flowPhase ? " visited" : "";
  const titleText = escapeHtml(shortText(title, 9));
  const bodyText = escapeHtml(shortText(text, 13));
  const rightPin = item.index !== 8;
  const leftPin = item.index !== 0;
  return `
    <g>
      <rect class="flow-node-box ${kind}${visited}${active}" x="${item.x}" y="${item.y}" width="${item.w}" height="${item.h}" rx="7"></rect>
      ${leftPin ? `<circle class="flow-pin exec" cx="${item.x}" cy="${item.y + 20}" r="4"></circle>` : ""}
      ${rightPin ? `<circle class="flow-pin exec" cx="${item.x + item.w}" cy="${item.y + 20}" r="4"></circle>` : ""}
      <circle class="flow-pin" cx="${item.x + 18}" cy="${item.y + item.h}" r="4"></circle>
      <text class="flow-title" x="${item.x + 12}" y="${item.y + 17}">${titleText}</text>
      <text class="flow-text" x="${item.x + 12}" y="${item.y + 31}">${bodyText}</text>
    </g>
  `;
}

function flowLine(from, to, kind) {
  const x1 = from.x + from.w;
  const y1 = from.y + from.h / 2;
  const x2 = to.x;
  const y2 = to.y + to.h / 2;
  return flowPath(`M ${x1} ${y1} C ${x1 + 38} ${y1}, ${x2 - 38} ${y2}, ${x2} ${y2}`, kind);
}

function flowPath(path, kind) {
  const marker =
    kind === "feedback" ? "arrowFeedback" :
    kind === "data" || kind === "loop" ? "arrowData" :
    "arrowExec";
  return `<path class="flow-link ${kind}" d="${path}" marker-end="url(#${marker})"></path>`;
}

function flowBadge(value, x, y) {
  return `
    <g>
      <circle class="flow-badge" cx="${x}" cy="${y}" r="9"></circle>
      <text class="flow-badge-text" x="${x}" y="${y + 0.5}">${value}</text>
    </g>
  `;
}

function shortText(value, maxLength) {
  const text = String(value);
  return text.length > maxLength ? `${text.slice(0, maxLength - 1)}…` : text;
}

function renderLegacyFlow(flow) {
  el.flowNodes.innerHTML = flow.nodes.map((node, index) => {
    const [kind, title, text] = node;
    const active = index === state.flowPhase ? " active" : "";
    return `
      <div class="flow-node legacy ${kind}${active}">
        <div class="flow-node-title">${escapeHtml(title)}</div>
        <div class="flow-node-text">${escapeHtml(text)}</div>
      </div>
    `;
  }).join("");
}

function modeName() {
  if (state.mode === "nmpc") return t("modeNmpc");
  if (state.mode === "mpc") return t("modeMpc");
  if (state.mode === "lqr") return t("modeLqr");
  return t("modePid");
}

function renderMetrics() {
  const stable = isStableAtTarget(params());
  const stableRecording = stable &&
    state.stableSince !== null &&
    state.t - state.stableSince < STABLE_TAIL_SECONDS;
  el.modeTitle.textContent = modeName();
  el.statusText.textContent = ` ${stableRecording ? t("stableRecording") : stable ? t("stable") : state.running ? t("running") : t("paused")}`;
  el.xValue.textContent = state.x.toFixed(2);
  el.vValue.textContent = state.v.toFixed(2);
  el.uValue.textContent = state.u.toFixed(2);
  el.costValue.textContent = Number.isFinite(state.cost) ? state.cost.toFixed(2) : "inf";
  el.candValue.textContent = String(state.candidates);
  el.pauseBtn.textContent = state.running ? t("pause") : t("resume");
  el.pauseBtn.classList.toggle("primary", state.running);
  el.predictiveTitle.textContent = state.mode === "mpc" ? t("mpcParams") : t("nmpcParams");
  el.advancedPredictiveTitle.textContent = state.mode === "mpc" ? t("mpcAdvancedParams") : t("nmpcAdvancedParams");
  el.nmpcMode.classList.toggle("active", state.mode === "nmpc");
  el.mpcMode.classList.toggle("active", state.mode === "mpc");
  el.lqrMode.classList.toggle("active", state.mode === "lqr");
  el.pidMode.classList.toggle("active", state.mode === "pid");
  const predictiveMode = state.mode === "nmpc" || state.mode === "mpc";
  el.predictiveControls.classList.toggle("hidden", !predictiveMode);
  el.advancedPredictiveControls.classList.toggle("hidden", !predictiveMode);
  el.pidControls.classList.toggle("hidden", state.mode !== "pid");
  el.lqrControls.classList.toggle("hidden", state.mode !== "lqr");
}

function frame() {
  updateLabels();
  if (state.running) {
    const p = params();
    if (!isStableTailComplete(p)) {
      state.speedRemainder += p.speed;
      const stepsThisFrame = Math.min(24, Math.floor(state.speedRemainder));
      state.speedRemainder -= stepsThisFrame;
      for (let i = 0; i < stepsThisFrame; i++) {
        if (isStableTailComplete(params())) break;
        step();
      }
    } else {
      state.speedRemainder = 0;
      state.flowPhase = FLOW_PHASE_COUNT - 1;
    }
  }
  renderMetrics();
  drawWorld();
  drawChart();
  renderFlow();
  requestAnimationFrame(frame);
}

function toggleRunning() {
  state.running = !state.running;
}

el.pauseBtn.addEventListener("click", toggleRunning);
el.resetBtn.addEventListener("click", reset);
el.resetParamsBtn.addEventListener("click", resetParams);
el.resetAllBtn.addEventListener("click", resetAll);
el.infoBtn.addEventListener("click", openInfoModal);
el.infoCloseBtn.addEventListener("click", closeInfoModal);
el.infoModal.addEventListener("click", event => {
  if (event.target === el.infoModal) closeInfoModal();
});
el.languageSelect.addEventListener("change", event => {
  applyLanguage(event.target.value);
});
el.nmpcMode.addEventListener("click", () => {
  state.mode = "nmpc";
  resetAll();
});
el.mpcMode.addEventListener("click", () => {
  state.mode = "mpc";
  resetAll();
});
el.lqrMode.addEventListener("click", () => {
  state.mode = "lqr";
  resetAll();
});
el.pidMode.addEventListener("click", () => {
  state.mode = "pid";
  resetAll();
});

for (const input of document.querySelectorAll("input[type='range']")) {
  input.addEventListener("input", updateLabels);
}

window.addEventListener("keydown", event => {
  if (event.code === "Escape" && !el.infoModal.hidden) {
    closeInfoModal();
    return;
  }
  if (event.code !== "Space" || event.repeat || !el.infoModal.hidden) return;
  event.preventDefault();
  toggleRunning();
});

if (!CanvasRenderingContext2D.prototype.roundRect) {
  CanvasRenderingContext2D.prototype.roundRect = function (x, y, w, h, r) {
    const radius = Math.min(r, w / 2, h / 2);
    this.beginPath();
    this.moveTo(x + radius, y);
    this.lineTo(x + w - radius, y);
    this.quadraticCurveTo(x + w, y, x + w, y + radius);
    this.lineTo(x + w, y + h - radius);
    this.quadraticCurveTo(x + w, y + h, x + w - radius, y + h);
    this.lineTo(x + radius, y + h);
    this.quadraticCurveTo(x, y + h, x, y + h - radius);
    this.lineTo(x, y + radius);
    this.quadraticCurveTo(x, y, x + radius, y);
    this.closePath();
  };
}

resetAll();
applyLanguage(initialLanguage());
requestAnimationFrame(frame);
