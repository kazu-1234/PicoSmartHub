// スクリプトのプロパティ（一時的なデータ保存領域）にデータを保存するためのキー
const CMD_KEY = 'WOL_COMMAND';
const TRIGGER_KEY = 'WOL_TRIGGER'; // 「信号」を保存するためのキー

// --- ↓↓↓ ユーザー設定項目 ↓↓↓ ---
// LINE Developersコンソールで取得したチャネルアクセストークン
const LINE_CHANNEL_ACCESS_TOKEN = 'YOUR_LINE_CHANNEL_ACCESS_TOKEN_HERE'; // ★★★ 自分のアクセストークンに書き換える ★★★
// --- ↑↑↑ ユーザー設定項目 ↑↑↑ ---

/**
 * @brief LINEプラットフォームからWebhookでPOSTリクエストを受け取ったときに実行される関数
 */
function doPost(e) {
  try {
    const event = JSON.parse(e.postData.contents).events[0];

    // テキストメッセージであり、特定のコマンドと一致する場合
    if (event.type === 'message' && event.message.type === 'text') {
      const messageText = event.message.text.trim();
      if (messageText === 'デスクトップPC起動' || messageText === 'サーバーPC起動') {
        
        // 1. 具体的なコマンドを保存する
        PropertiesService.getScriptProperties().setProperty(CMD_KEY, messageText);
        // 2. 「信号あり」のフラグを立てる
        PropertiesService.getScriptProperties().setProperty(TRIGGER_KEY, '1');

        // ★★★ 追加機能: LINEに応答メッセージを返す ★★★
        // これがないとLINE上では何も反応がないように見えるため、ユーザー体験が向上します。
        replyMessage(event.replyToken, `${messageText}しました。`);

      }
    }
  } catch (err) {
    // エラーが発生した場合にログを記録する
    console.error("doPost Error: " + err.toString() + " Stack: " + err.stack);
  }
  
  // LINEプラットフォームには必ず200 OKを返す
  return ContentService.createTextOutput(JSON.stringify({'status': 'ok'})).setMimeType(ContentService.MimeType.JSON);
}

/**
 * @brief Raspberry Pi Pico WからGETリクエストを受け取ったときに実行される関数
 */
function doGet(e) {
  try {
    const query = e.parameter.action; 

    // 1. 「信号はありますか？」という問い合わせの場合
    if (query === 'signal') {
      const trigger = PropertiesService.getScriptProperties().getProperty(TRIGGER_KEY);
      if (trigger === '1') {
        return ContentService.createTextOutput('TRIGGER'); // 信号があれば "TRIGGER" と返信
      }
      return ContentService.createTextOutput(''); // なければ空文字
    }
    
    // 2. 「具体的なコマンドは何ですか？」という問い合わせの場合
    if (query === 'command') {
      const scriptProperties = PropertiesService.getScriptProperties();
      const command = scriptProperties.getProperty(CMD_KEY);

      if (command) {
        // コマンドを返した後、コマンドと信号の両方を削除して多重実行を防ぐ
        scriptProperties.deleteProperty(CMD_KEY);
        scriptProperties.deleteProperty(TRIGGER_KEY);
        return ContentService.createTextOutput(command);
      }
      return ContentService.createTextOutput('');
    }
  } catch (err) {
    console.error("doGet Error: " + err.toString() + " Stack: " + err.stack);
    return ContentService.createTextOutput('ERROR');
  }

  // パラメータが不正な場合は空文字を返す
  return ContentService.createTextOutput('');
}


/**
 * @brief LINEに応答メッセージを送信する関数
 * @param {string} replyToken - 応答用のトークン
 * @param {string} message - 送信するテキストメッセージ
 */
function replyMessage(replyToken, message) {
  const url = 'https://api.line.me/v2/bot/message/reply';
  
  const headers = {
    'Content-Type': 'application/json',
    'Authorization': 'Bearer ' + LINE_CHANNEL_ACCESS_TOKEN,
  };
  
  const payload = {
    'replyToken': replyToken,
    'messages': [{
      'type': 'text',
      'text': message,
    }],
  };
  
  const options = {
    'method': 'post',
    'headers': headers,
    'payload': JSON.stringify(payload),
    'muteHttpExceptions': true, // エラー時にも例外を投げない
  };
  
  try {
    UrlFetchApp.fetch(url, options);
  } catch (err) {
    console.error("Reply message failed: " + err.toString());
  }
}
