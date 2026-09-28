const MESSAGE_TYPES = ["success", "warning", "info", "error"];
const DEFAULT_OFFSET = 20;
const MESSAGE_GAP = 16;

function normalizeOptions(options, type) {
  const messageOptions =
    typeof options === "object" &&
    options !== null &&
    !hasOwn(options, "componentOptions")
      ? { ...options }
      : { message: options };

  if (type) {
    messageOptions.type = type;
  }

  return messageOptions;
}

function hasOwn(object, key) {
  return Object.prototype.hasOwnProperty.call(object, key);
}

export default function createMessage(originalMessage) {
  const activeMessages = {};
  let messageInstances = [];
  let reflowTimer = null;

  const reflowMessages = () => {
    let verticalOffset = DEFAULT_OFFSET;
    messageInstances = messageInstances.filter(
      instance => instance && !instance.closed
    );

    messageInstances.forEach(instance => {
      const messageOffset = instance.messageOffset || DEFAULT_OFFSET;
      verticalOffset = Math.max(verticalOffset, messageOffset);
      instance.verticalOffset = verticalOffset;
      verticalOffset += instance.$el.offsetHeight + MESSAGE_GAP;
    });
  };

  const scheduleReflow = () => {
    if (reflowTimer) {
      return;
    }

    reflowTimer = setTimeout(() => {
      reflowTimer = null;
      reflowMessages();
    }, 0);
  };

  const trackInstance = (instance, messageOptions) => {
    if (!instance) {
      return;
    }

    instance.messageOffset = messageOptions.offset || DEFAULT_OFFSET;
    messageInstances.push(instance);
    scheduleReflow();
  };

  const removeInstance = instance => {
    messageInstances = messageInstances.filter(item => item !== instance);
    scheduleReflow();
  };

  const createOnClose = id => instance => {
    removeInstance(instance);
    const activeMessage = activeMessages[id];
    if (!activeMessage || activeMessage.instance !== instance) {
      return;
    }

    delete activeMessages[id];
    if (typeof activeMessage.onClose === "function") {
      activeMessage.onClose(instance);
    }
  };

  const updateMessage = (id, messageOptions) => {
    const activeMessage = activeMessages[id];
    const instance = activeMessage && activeMessage.instance;

    if (!instance || instance.closed) {
      return null;
    }

    const updateKeys = [
      "message",
      "type",
      "iconClass",
      "customClass",
      "duration",
      "showClose",
      "dangerouslyUseHTMLString",
      "center"
    ];

    updateKeys.forEach(key => {
      if (hasOwn(messageOptions, key)) {
        instance[key] = messageOptions[key];
      }
    });

    if (hasOwn(messageOptions, "onClose")) {
      activeMessage.onClose = messageOptions.onClose;
      instance.onClose = createOnClose(id);
    }

    if (typeof instance.clearTimer === "function") {
      instance.clearTimer();
    }
    if (typeof instance.startTimer === "function") {
      instance.startTimer();
    }

    scheduleReflow();
    return instance;
  };

  const createNormalOnClose = userOnClose => instance => {
    removeInstance(instance);
    if (typeof userOnClose === "function") {
      userOnClose(instance);
    }
  };

  const showMessage = options => {
    const messageOptions = normalizeOptions(options);
    const id = messageOptions.id;

    if (id === undefined || id === null || id === "") {
      delete messageOptions.id;
      const userOnClose = messageOptions.onClose;
      messageOptions.onClose = createNormalOnClose(userOnClose);
      const instance = originalMessage(messageOptions);
      trackInstance(instance, messageOptions);
      return instance;
    }

    const messageId = String(id);
    delete messageOptions.id;

    const updatedInstance = updateMessage(messageId, messageOptions);
    if (updatedInstance) {
      return updatedInstance;
    }

    activeMessages[messageId] = {
      instance: null,
      onClose: messageOptions.onClose
    };
    messageOptions.onClose = createOnClose(messageId);

    const instance = originalMessage(messageOptions);
    activeMessages[messageId].instance = instance;
    trackInstance(instance, messageOptions);
    return instance;
  };

  MESSAGE_TYPES.forEach(type => {
    showMessage[type] = options => showMessage(normalizeOptions(options, type));
  });

  showMessage.close = (id, onClose) => {
    const messageId = String(id);
    const activeMessage = activeMessages[messageId];
    if (activeMessage && activeMessage.instance) {
      if (typeof onClose === "function") {
        activeMessage.onClose = onClose;
      }
      activeMessage.instance.close();
      return;
    }

    if (typeof originalMessage.close === "function") {
      originalMessage.close(id, onClose);
    }
  };

  showMessage.closeAll = () => {
    if (typeof originalMessage.closeAll === "function") {
      originalMessage.closeAll();
    }
  };

  return showMessage;
}
